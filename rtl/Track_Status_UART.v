`timescale 1ns/1ps
// V3 serializes packet bodies into one 128x8 RAM10 (including a six-target frame). Target copies abort
// rather than stall video if the existing track bank changes during capture.
// REMOTE_ENABLE=0 keeps the original immediate v2 snapshot contract for benches.
module Track_Status_UART #(parameter TARGETS=6,CLOCK_HZ=96000000,BAUD=2000000,REMOTE_ENABLE=0,FRAME_STREAM_ENABLE=0)(
    input wire clk,rst_n,publish,
    input wire [TARGETS*51-1:0] boxes,
    input wire [TARGETS*4-1:0] digits,
    input wire [TARGETS*2-1:0] qualities,
    input wire [TARGETS*8-1:0] lost,ids,ages,
    input wire [TARGETS*13-1:0] vx,vy,
    input wire [TARGETS-1:0] alive,observed,
    input wire [9:0] fps_x10,
    input wire [15:0] errors,
    input wire source_safe,source_tag,
    output wire tx,busy,
    input wire request_valid,
    input wire [7:0] request_sequence,request_command,
    input wire [2:0] request_status,
    input wire [1:0] request_kind,
    input wire [15:0] request_value,
    input wire [127:0] raw_data,
    output wire response_ready,
    output reg [7:0] overruns
);
    localparam DIVISOR=CLOCK_HZ/BAUD,TB=DIVISOR<=1?1:$clog2(DIVISOR),IB=$clog2(TARGETS+1);
    reg [IB-1:0] slot;
    reg [9:0] serial_shift;
    reg [TB-1:0] timer;
    reg [3:0] bit_number;
    reg [6:0] byte_number,last_byte;
    reg [7:0] checksum,packet_type,active_sequence;
    reg sending,loading,pending,reject_pending;
    reg [7:0] reply_sequence,reply_command,reject_sequence,reject_command;
    reg [15:0] reply_value;
    reg [2:0] reply_status;
    reg [1:0] reply_kind;
    reg filling,fill_reply,fill_reject,fill_target,fill_tag,fill_abort,force_busy;
    reg [6:0] fill_index,fill_last,body_index;
    reg [3:0] record_byte;
    reg [2:0] frame_slot;
    reg frame_pending,fill_frame;
    reg [15:0] frame_counter,status_frame;
    wire take_frame=REMOTE_ENABLE && FRAME_STREAM_ENABLE && !busy && !pending &&
        !reject_pending && !request_valid && frame_pending && source_safe;
    reg [9:0] status_fps;
    reg [15:0] status_errors;
    reg [2:0] status_count;
    wire take_reply=REMOTE_ENABLE && !busy && pending;
    wire take_reject=REMOTE_ENABLE && !busy && !pending && reject_pending;
    assign response_ready=!pending;
    wire query_in_progress=filling && packet_type[7:3]==5'b00101;
    wire [IB-1:0] selected_slot=fill_frame && filling?frame_slot:(take_reply && reply_kind==2 || query_in_progress)?reply_value[IB-1:0]:slot;
    reg [119:0] target_record;
    reg [7:0] target_count;
    integer k;
    always @(*) begin
        target_count=0;for(k=0;k<TARGETS;k=k+1) target_count=target_count+observed[k];
    end
    integer read_slot;
    always @(*) begin
        target_record=0;
        for(read_slot=0;read_slot<TARGETS;read_slot=read_slot+1)
            target_record=target_record | ({
                {3{vy[read_slot*13+12]}},vy[read_slot*13 +: 13],
                {3{vx[read_slot*13+12]}},vx[read_slot*13 +: 13],boxes[read_slot*51 +: 48],
                lost[read_slot*8 +: 8],ages[read_slot*8 +: 8],4'd0,digits[read_slot*4 +: 4],
                ids[read_slot*8 +: 8],alive[read_slot],2'b00,qualities[read_slot*2 +: 2],boxes[read_slot*51+48 +: 3]
            } & {120{selected_slot==read_slot}});
    end
    wire telemetry_request=!FRAME_STREAM_ENABLE && publish && !(REMOTE_ENABLE && request_valid) &&
        (!REMOTE_ENABLE || slot==TARGETS || source_safe);
    wire start_packet=!busy && (take_reply || take_reject || take_frame || telemetry_request);
    reg [119:0] load_body;
    reg [7:0] load_type;
    reg [6:0] load_last;
    reg [6:0] load_fill_last;
    reg load_target;
    always @(*) begin
        load_body=target_record;load_type=8'h20 | slot;load_last=17;load_fill_last=14;load_target=1;
        if(take_reject) begin
            load_body={96'd0,8'd4,reject_command,reject_sequence};load_type=8'h90;load_last=5;load_fill_last=2;load_target=0;
        end else if(take_reply) begin
            if(reply_kind==2 && source_safe) begin load_type=8'h28 | selected_slot;load_last=18;end
            else if(reply_kind==1) begin
                load_body={88'd0,reply_value,reply_command,reply_sequence};load_type=8'h91;load_last=6;load_fill_last=3;load_target=0;
            end else if(reply_kind==3) begin
                load_type=8'ha1;load_last=23;load_fill_last=20;load_target=0;
            end else begin
                load_body={96'd0,5'd0,(reply_kind==2?3'd4:reply_status),reply_command,reply_sequence};
                load_type=8'h90;load_last=5;load_fill_last=2;load_target=0;
            end
        end else if(take_frame) begin
            load_type=8'ha0;load_last=99;load_fill_last=96;load_target=1;
        end else if(slot==TARGETS) begin
            load_body={72'd0,(REMOTE_ENABLE?8'd3:8'd2),target_count,errors,6'd0,fps_x10};
            load_type=8'h81;load_last=8;load_fill_last=5;load_target=0;
        end
    end
    reg [7:0] fill_byte;
    reg [15:0] raw_crc;
    function [15:0] crc16;
        input [15:0] old_crc;
        input [7:0] value;
        reg [15:0] c;
        integer j;
        begin
            c=old_crc^{value,8'd0};
            for(j=0;j<8;j=j+1) c=c[15]?(c<<1)^16'h1021:c<<1;
            crc16=c;
        end
    endfunction
    always @(posedge clk) begin
        if(start_packet) raw_crc<=16'hffff;
        else if(filling && packet_type==8'ha1 && fill_index<19) raw_crc<=crc16(raw_crc,fill_byte);
    end
    always @(*) begin
        fill_byte=target_record[(fill_frame?record_byte:fill_index)*8 +: 8];
        if(fill_frame) begin
            case(fill_index)
                0:fill_byte=status_frame[7:0];
                1:fill_byte=status_frame[15:8];
                2:fill_byte={5'd0,status_count};
                93:fill_byte=status_fps[7:0];
                94:fill_byte={6'd0,status_fps[9:8]};
                95:fill_byte=status_errors[7:0];
                96:fill_byte=status_errors[15:8];
                default:begin end
            endcase
        end
        if(packet_type==8'ha1) begin
            case(fill_index)
                0:fill_byte=reply_sequence;
                1:fill_byte=reply_value[7:0];
                2:fill_byte=reply_value[15:8];
                19:fill_byte=raw_crc[7:0];
                20:fill_byte=raw_crc[15:8];
                default:fill_byte=raw_data[(fill_index-3)*8 +: 8];
            endcase
        end else if(packet_type==8'h90 || packet_type==8'h91) begin
            case(fill_index)
                0:fill_byte=fill_reject?reject_sequence:reply_sequence;
                1:fill_byte=fill_reject?reject_command:reply_command;
                2:fill_byte=packet_type==8'h91 ? reply_value[7:0]:
                    {5'd0,(fill_reject || force_busy?3'd4:reply_status)};
                default:fill_byte=reply_value[15:8];
            endcase
        end else if(packet_type==8'h81) begin
            case(fill_index)
                0:fill_byte=status_fps[7:0];
                1:fill_byte={6'd0,status_fps[9:8]};
                2:fill_byte=status_errors[7:0];
                3:fill_byte=status_errors[15:8];
                4:fill_byte={5'd0,status_count};
                default:fill_byte=3;
            endcase
        end
    end
    wire query_target=REMOTE_ENABLE && packet_type[7:3]==5'b00101;
    wire insert_sequence=query_target && byte_number==2;
    wire [7:0] body_byte;
    wire shift_body=loading && byte_number!=last_byte && byte_number>=2 && !insert_sequence;
    generate if(REMOTE_ENABLE) begin:g_packet_ram
        (* syn_ramstyle="block_ram" *) reg [7:0] data[0:127];
        reg [7:0] q;
        always @(posedge clk) begin
            if(filling) data[fill_index]<=fill_byte;
            q<=data[body_index];
        end
        assign body_byte=q;
    end else begin:g_legacy_body
        reg [119:0] body;
        always @(posedge clk) if(start_packet) body<=load_body;else if(shift_body) body<=body>>8;
        assign body_byte=body[7:0];
    end endgenerate
    wire [7:0] next_byte=byte_number==0?8'ha5:byte_number==1?packet_type:insert_sequence?active_sequence:body_byte;
    assign busy=sending || loading || (REMOTE_ENABLE && filling);
    assign tx=sending?serial_shift[0]:1'b1;
    wire fill_done=filling && fill_index==fill_last;
    wire invalid_copy=fill_target && (fill_abort || !source_safe || fill_tag!=source_tag || (fill_frame && publish));
    always @(posedge clk) begin
        if(start_packet) begin
            packet_type<=load_type;last_byte<=load_last;active_sequence<=reply_sequence;
            status_frame<=frame_counter;status_fps<=fps_x10;status_errors<=errors;status_count<=target_count[2:0];
        end else if(fill_done && invalid_copy && fill_reply) begin packet_type<=8'h90;last_byte<=5;end
        if(REMOTE_ENABLE && request_valid) begin
            if(response_ready) begin
                reply_sequence<=request_sequence;reply_command<=request_command;reply_value<=request_value;
                reply_status<=request_status;reply_kind<=request_status==0?request_kind:2'd0;
            end else if(!reject_pending) begin reject_sequence<=request_sequence;reject_command<=request_command;end
        end
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            slot<=0;serial_shift<=10'h3ff;timer<=0;bit_number<=0;byte_number<=0;checksum<=0;sending<=0;loading<=0;
            pending<=0;reject_pending<=0;overruns<=0;filling<=0;fill_reply<=0;fill_reject<=0;fill_target<=0;
            frame_pending<=0;frame_counter<=0;fill_frame<=0;frame_slot<=0;record_byte<=0;
            fill_tag<=0;fill_abort<=0;force_busy<=0;fill_index<=0;fill_last<=0;body_index<=0;
        end else begin
            if(FRAME_STREAM_ENABLE && publish) begin frame_pending<=1;frame_counter<=frame_counter+1'b1;end
            if(REMOTE_ENABLE && request_valid) begin
                if(response_ready) pending<=1;
                else if(!reject_pending) reject_pending<=1;
                else overruns<=8'd1; // Sticky overdrive flag; no extra byte counter.
            end
            if(start_packet) begin
                checksum<=0;byte_number<=0;body_index<=0;
                if(REMOTE_ENABLE) begin
                    filling<=1;fill_index<=0;fill_frame<=take_frame;frame_slot<=0;record_byte<=0;fill_last<=load_fill_last;fill_reply<=take_reply;fill_reject<=take_reject;
                    fill_target<=load_target;fill_tag<=source_tag;fill_abort<=0;force_busy<=take_reply && reply_kind==2 && !source_safe;
                end else begin loading<=1;slot<=slot==TARGETS?0:slot+1'b1;end
            end else if(REMOTE_ENABLE && filling) begin
                if(fill_target && (!source_safe || fill_tag!=source_tag || (fill_frame && publish))) fill_abort<=1;
                if(fill_done) begin
                    if(invalid_copy && fill_reply) begin
                        fill_index<=0;fill_last<=2;fill_target<=0;force_busy<=1;
                    end else begin
                        filling<=0;
                        if(fill_reply) pending<=0;
                        if(fill_reject) reject_pending<=0;
                        if(fill_frame && !invalid_copy && !publish) frame_pending<=0;
                        if(!fill_reply && !fill_reject && !fill_frame) slot<=slot==TARGETS?0:slot+1'b1;
                        if(!invalid_copy) loading<=1; // Drop a torn passive record.
                    end
                end else begin
                    fill_index<=fill_index+1'b1;
                    if(fill_frame && fill_index>=3 && fill_index<=92) begin
                        if(record_byte==14) begin record_byte<=0;frame_slot<=frame_slot+1'b1;end
                        else record_byte<=record_byte+1'b1;
                    end
                end
            end else if(loading) begin
                serial_shift<={1'b1,byte_number==last_byte?checksum:next_byte,1'b0};
                if(byte_number!=last_byte) checksum<=checksum^next_byte;
                if(shift_body) body_index<=body_index+1'b1;
                bit_number<=0;timer<=DIVISOR-1;sending<=1;loading<=0;
            end else if(timer!=0) timer<=timer-1'b1;
            else if(sending && bit_number==9) begin sending<=0;if(byte_number!=last_byte) begin byte_number<=byte_number+1'b1;loading<=1;end end
            else if(sending) begin bit_number<=bit_number+1'b1;serial_shift<={1'b1,serial_shift[9:1]};timer<=DIVISOR-1;end
        end
    end
    // synthesis translate_off
    initial if(DIVISOR<2 || TARGETS<1 || TARGETS>8 || (FRAME_STREAM_ENABLE && TARGETS!=6)) $fatal(1,"Invalid UART parameters");
    // synthesis translate_on
endmodule
