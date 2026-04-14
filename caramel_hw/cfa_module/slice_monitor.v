module  slice_monitor(
    clk,
    //
    pc,
    pc_nxt,
    //
    ER_min,
    ER_max,
    //
    ER_done,
    irq,
    reset,

	cflow_log_ptr,

    flush_T2, 
	slice_top,
	slice_bot


);

input		clk;//clock
input   [15:0]  pc;//program counter
input   [15:0]  pc_nxt;//counter in next step?
//
input   [15:0]  ER_min;//start and end of CFlog
input   [15:0]  ER_max;

//
input       ER_done;
input       irq; //interrup
input		reset;//reset
input  [15:0]  cflow_log_ptr;//Pointer for: Where in the Cflog are we?

output 			flush_T2;	//T2 internally Slice is ready for transmission, triggers the TCB
output	[15:0]  slice_top;
output	[15:0]  slice_bot;

////////////////////////////////////////////////// Parameters and Variables  //////////////////////////////////////////////////////////

parameter TCB_max = 16'hdffe;

// Trigger States
parameter VrfHold  = 1'b0;
parameter EXEC = 1'b1;


parameter LOG_SIZE = 16'h0000; // Size of the cflog gets overwritten by parent module
parameter NUM_SLICES = 2; // number of slices as in the paper here two but it can be changed
parameter MAX_CFLOG_SIZE = LOG_SIZE-2; // the end of the cflog, set as minus 2 to log tcb entry, set as equal to ignore tcb entry
parameter SLICE_SIZE = LOG_SIZE/NUM_SLICES; // size of a single Cflog slice, dependend on the number of slices

//-------------Internal Variables---------------------------
reg             trigger_state;
reg             flush_slice_T2; //Internal handler for T2 trigger
reg             attest_pend;
reg             res = 1'b1;


reg     [15:0]  bound_low; // end of slice currently written to
reg     [15:0]  top_of_slice_reg;
reg		[15:0]	slice_bottom; //slice that was or is verified
reg     [2:0]	where_is_log_ptr;

//

initial
begin
    trigger_state = EXEC;
    flush_slice_T2 = 1'b0;
    attest_pend = 1'b0;
	slice_bottom = 1'b0;//intializing with all 0000
	where_is_log_ptr = 1'b0;// which slice is currently or should be getting written to
	bound_low = 16'b0+SLICE_SIZE;//-2'b10;
	top_of_slice_reg = 16'b0;
end


////////////////////////////////////////////////// Internal Logic  //////////////////////////////////////////////////////////

wire cflog_ptr_next = (cflow_log_ptr + 16'h2) % (LOG_SIZE+1);
wire slice_full = (cflow_log_ptr >= bound_low && cflow_log_ptr <= (bound_low+2'b10)) && ~finished || ((cflow_log_ptr == MAX_CFLOG_SIZE) && slice_top==16'b0);// update current version doesn't work if cfptr is at end and 
wire vrf_acc = pc==16'hdffc; //trampoline address fro when Vrf response was accept


//system state: or are we writing to cflog or are we waiting on vrf?
always @(posedge clk)  
if(( trigger_state == EXEC && slice_full) || ( pc == ER_max && pc != 1'b0))// we have a accepted vrf response and the current slice is ready for transmission or execution is finished [T1]
    trigger_state <= VrfHold;
else if (trigger_state == VrfHold && vrf_acc)// we send of a slice and are currentlz waiting for vrf to respond
    trigger_state <= EXEC;
else trigger_state <= trigger_state;

// support logic incase of race conditions
reg finished = 1'b0;
always @(posedge clk)
begin
    if(ER_done)
        finished <= 1'b1;  
    else
        finished <= finished;  
end


// Slice boundries update and T2 trigger logic
always @(posedge clk)
if (trigger_state == EXEC && slice_full)// Vrf_acc and slice_full
    begin
    flush_slice_T2 <= 1'b1; //trigger T2
	where_is_log_ptr = (where_is_log_ptr+1) % NUM_SLICES; //update when leaving tcb through dff6
	slice_bottom = bound_low; //set slice boundaries for transmission
	bound_low = SLICE_SIZE*(where_is_log_ptr+1);// update writing boundaries 
	res<=1'b1; // currently in the process of sending supporting logic bc of race condition
    end
else if (trigger_state == VrfHold && slice_full && ( cflog_ptr_next!= slice_top)) // slice_full and log_full 
    begin
    flush_slice_T2 <= 1'b0; //ensure T2 is not set
	where_is_log_ptr = (where_is_log_ptr+1)% NUM_SLICES; 
	bound_low = SLICE_SIZE*(where_is_log_ptr+1); // update internal boundaries for when we return to writing
	end
else if (attest_pend) //Pend --> Acc
    flush_slice_T2 <= 0;
else if ((vrf_acc && ER_min != 0) && res) //Vrf response was accepted, update slice top boundary for next transmission
    begin
    top_of_slice_reg = (slice_bottom+16'b10) % (LOG_SIZE+2'b10);
    res <=1'b0;
    end
else if (finished && res) //ER_done or T1 happened prepare boundaries for the current slice that is send
begin
    top_of_slice_reg = (slice_bottom+16'b10) % (LOG_SIZE+2'b10);
    res <=1'b0;
    slice_bottom = cflow_log_ptr;
end

    
    
always @(posedge clk)
if(flush_slice_T2 && irq)// Abort --> Pend
    attest_pend <= 1;
else
    attest_pend <= 0; 
    

////////////////////////////////////////////////// OUTPUT Logic  //////////////////////////////////////////////////////////

assign flush_T2 = flush_slice_T2; // rename 
assign slice_top = top_of_slice_reg;
assign slice_bot = slice_bottom;

endmodule

