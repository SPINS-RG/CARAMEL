`include "openMSP430_defines.v"
module  controller (
    mclk,
    pc,
    cflow_log_ptr,
    read_val,
    boot,///don't think it's needed here
    //flush_log_T4,
    flush_slice_T2,
    ER_done,
    vrf_response_out,
    slice_top,
    slice_bot,

    //
    start_log,
    trigger,
    read_idx,
    continue,
    byte_val,

    t1_pend,
    t2_pend,
    //t3,

    log_ptr_catch_out,
    
    log_idx, //where in logmem are we reading
    read_val_log      // Data memory data output for hardware is connected to controller (uut)
);

input mclk;
input [15:0] pc;
input [15:0] cflow_log_ptr;
input [15:0] read_val;
input boot;
//input flush_log_T4;
input flush_slice_T2;
input ER_done;
input [15:0] vrf_response_out;
input [15:0] slice_top;
input [15:0] slice_bot;

output reg trigger;
output [15:0] read_idx;
output continue;
output [7:0] byte_val;


//// debug
output reg t1_pend;
output reg t2_pend;
//output reg t3;

output reg [15:0] log_ptr_catch_out;

//logmem

input  [15:0]       read_val_log;
output [`LOG_MSB:0] log_idx;

////////////////////////////////////////////////// Parameters and Variables  //////////////////////////////////////////////////////////

// Number of ticks between byte transmisions
// CLK_freq / BAUD 
// 20*10^6 / 112500
// ~180
parameter CLK_freq = 20000000;

//BAUD rate can be modified in the openMSP430_defines.v file
parameter TX_RATE = CLK_freq/`CARAMEL_BAUD;

//only if zou want to hardcode the transmission rate
//parameter TX_RATE = 180; // transmission rate

//Transmission sizes as in caramel memory
parameter CHAL_SIZE_WORDS  =  16'h10; 
parameter MAC_WORDS  =  16'h10; 
parameter METADATA_SIZE_WORDS  =  16'h7; 

parameter LOG_SIZE = `LOG_SIZE/2; // now updated to be dependent on defines :)

// sum of transmitted Meta data
parameter CONSTANTS = CHAL_SIZE_WORDS+MAC_WORDS+METADATA_SIZE_WORDS;

//These signals track what report we are currently sending. T2 and t3 can occure simultanously this way one isnt't accidentially lost :)
parameter NONE     =  2'b00; 
parameter T2_GEN   =  2'b01;  
parameter T3_GEN   =  2'b10; 


//-------------Internal Variables--------------------------
reg counter;
reg repo_wait;
reg [15:0] log_ptr_catch;
output reg start_log;
reg [15:0] trigger_count;

// internal handling of slice boundaries
reg [15:0] bottom_catch = 0;
reg [15:0] top_catch = 0;

 //initilaising new stuff
reg [15:0] read_idx_reg; // old one as the counting is the same we wont change that (it will just get updated in between)
reg start; 
reg log_finish; //finished sending the log so we can start sending anything else 


initial
begin
    // 
    t1_pend <= 1'b0;
    t2_pend <= 1'b0;
//    t3 <= 1'b0;
    log_ptr_catch <= 16'h0;
    start <= 1'b0;
    counter <= 0;
    trigger_count <= 16'h0;
    trigger <= 1'b0;
    log_ptr_catch_out <= 16'h0;
    repo_wait<= 1'b0;
    start_log<= 1'b0;
    log_finish<= 1'b0;
    read_idx_reg <=1'b0;
end

////////////////////////////////////////////////// Internal Logic  //////////////////////////////////////////////////////////



// triggers to start transmitting 
reg [1:0] tx_src = NONE; // 00 - t1_pend, 01 -- t2_pend, 10 -- t3


/// Register report transmission due to [T2] and [T1]
always @(posedge (mclk))
begin
    if (ER_done)
        t1_pend <= 1'b1;  
    else if (t1_pend & (start == 1'b1) && (read_idx > CONSTANTS) && (tx_src == T3_GEN)) //
        t1_pend <= 1'b0; 
    if(flush_slice_T2)
        t2_pend <= 1'b1;
    else if (t2_pend & start && (read_idx > CONSTANTS) && (tx_src == T2_GEN))
        t2_pend <= 1'b0;  
end 



// used to detect when vrf has approved the previous report, and the current one can be sent.
reg vrf_acc = 1'b1;
reg first_tx = 1'b1;
reg report_ready = 1'b0;
wire is_accepted_addr = (pc == 16'hdffc);
wire is_sendoff_addr = (pc == 16'hdffa);
wire one_cflog = (~t2_pend & t1_pend & first_tx); /// if the whole exec fits into one cflog, 

//----- Permisiion to transmit e.g. was previous report accepted or is it the first trsnamission
always @(posedge mclk)
begin
    if (is_sendoff_addr && (~repo_wait | one_cflog) & ~start)
    begin
        vrf_acc <= ~first_tx;
        report_ready <= 1'b1;
        repo_wait<=1'b1;
    end
    else if (is_accepted_addr | (is_sendoff_addr & first_tx))
    begin
        vrf_acc <= 1'b1;
        first_tx <= 1'b0;
    end
    else if (~report_ready)
        repo_wait<= 1'b0;
end



/// flag to start transmitting 
wire tx_pending = (t2_pend | t1_pend); //Register report transmission due to [T2] and [T1]
wire pc_not_in_tcb = (pc < 16'ha000) | (pc > 16'hdffe); 
wire pc_in_tcb = ~pc_not_in_tcb;

reg sending_byte =0;// ensures both bytes in an address are send and no bytes are lost 
reg sending = 1'b0;// ensures we are not sending while waiting for the next vrf response (so we can only send after receiving a response)
// is needed as the vrf_response_out leaves to fast and does not coincite with a flush signal so while not the pretties and there are probablty otherways like solving it at the source pof vrf_resp signal this works

//------------- 
always @(posedge (mclk))
begin//so this starts when we are ready to send a report and finished sending the cflog 
    if (tx_pending && report_ready && ~start  && log_finish)//pc_not_in_tcb 
    begin
        report_ready <= 1'b0;
        start <= 1'b1;  
        read_idx_reg <= 1'b0;
        if (t2_pend)
            tx_src <= T2_GEN;
        else if (t1_pend)
            tx_src <= T3_GEN;
        else
            tx_src <= NONE;
    end
    else if ((start == 1'b1) &&  (read_idx > CONSTANTS) && ~ sending_byte) // finished sending everything after starting to send METADATA and reached the its end  //FORMER:(read_idx >= log_ptr_catch))
     begin
        start <= 1'b0;
        log_finish  <= 1'b0;
        end
    else if (start == 1'b0)
    begin ///should you be looking at the waves and wonder why bottom doesn't update? that is totally fine. its the top that changes first its a slice (top above bottom) then its the whole log (top below bottom)
        top_catch <= slice_top;
        if (slice_bot == LOG_SIZE)
            bottom_catch <= LOG_SIZE-1'b1;
        else
            bottom_catch <= slice_bot+1'b1;
    end
end


 always @(posedge (mclk))
 begin
    if (vrf_response_out != 0) 
            sending = 1'b1;
    else if (vrf_response_out == 0 && log_finish)
        sending = 1'b0;
 end
 

//----------------- sending cflog -------------------//
// starts sending cflog upon entering tcb due to triggers, T1, T2, T4 though T2 and T4  are treated the same as they occur together if a transmission is supposed to send.
// keeps the controller active if there is  pause between finish sending the cflog until the rest of teh report is ready
 always @(posedge (mclk))
 begin
    if (tx_pending && ~start_log && ~log_finish && sending ) //TODO: better signal for ready to send anything I stole Adams for now additionally for a slice sending can finish before hmac is ready so to stop retransmitting the log we can not have a finished send log
    begin 
        start_log <= 1'b1;// start sending 
        log_finish  <= 1'b0;
        read_idx_reg <= slice_top; //inititalize read_idx_reg with the top_slice as no matter what cflog we are sending this should be the start
        end
    else if (start_log && log_idx == bottom_catch && ~sending_byte && log_idx !=0) // we finish with the log if we first started (XD) and then reached the bottom (so finished sending the bottom) thins works 
    begin
       start_log <= 1'b0; 
        log_finish  <= 1'b1; //set log finished
         read_idx_reg <= 1'b0; //after we finish the log, reset the read_idx_reg for reading the metadata
        end 
 end
 
 
 
 
////////////////////////////////////////////////// SEND DATA //////////////////////////////////////////////////////////

// output mux for what word is being send (gets fed into the byte val for uart) 
// if the we are currently in the cflog sending phase so we started sending cflog but are not finished the out put value (current value) is the value read from logmem otherwis eits the value from acfa_memory
wire [15:0] current_val;
//if in the process of sending log its the value form logmem otherwise its metadata 
assign current_val = (start_log && ~log_finish)  ? read_val_log : read_val ; 

//what is the current log idx (read_addr) for logmem e.g. the wrappinmg increment for reading from the cflog
//logic: cflog always starts from the top independent of where bottom is (so log_idx is outomatically read_idx_reg+topslice) if it becomes larger than LOGSIZE we adjust it by substarcting the LOGSIZE so it would start counting from LOG beginning
assign log_idx = read_idx_reg <= LOG_SIZE? read_idx_reg: read_idx_reg - LOG_SIZE;

// readvalue for the meta data goes to caramel memory
assign read_idx =  read_idx_reg;

///signal for the actual uart doing the sending now ends when METADATA finished sending (becomes controller_en). 
assign continue = (start_log  | log_finish);// 


/// iterates through cflog entries and meta data entries byte by byte in each entry
// trigger_count is delay between bytes
always @(posedge (mclk))
begin
    if (start_log == 1'b1 | start == 1'b1) //report is ready start_log= sening cflog and start clfog is finished and hmac is calculated 
        begin
            if(trigger == 1'b1) // ready to sen the next byte?
            begin
                if(counter == 1'b1) // what part of the cflog entry are we sending. Memory is made up of words but uart sends bytes. so we send lower bits first then the upper bits 
                begin
                    sending_byte <= 1'b1;
                    read_idx_reg <= read_idx_reg + 16'h1;//after sending upper bytes is the next entry read
                    counter <= 1'b0;
                end
                else if(counter == 1'b0)
                    counter <= 1'b1;
                trigger <= 1'b0;
            end
    
            if (trigger_count != TX_RATE) ///essantially a wait until byte is send
                trigger_count <= trigger_count + 1; 
            else
            begin
                trigger_count <= 16'h0;///prepare sending next byte  set timmer (trigger count to zero) and trigger sending next byte
                trigger <= 1'b1;
                if(counter == 1'b1)
                    sending_byte <= 1'b0;
            end
        end
    else
    begin
      //  read_idx_reg <= 16'h0; // 
        counter <= 0;
        trigger_count <= 16'h0;
        trigger <= 1'b0;
    end
end 

////////////////////////////////////////////////// THE SEND BYTE  //////////////////////////////////////////////////////////

assign byte_val = counter ? current_val[15:8] : current_val[7:0];  //byte that is currently being send

endmodule