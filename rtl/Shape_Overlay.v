`timescale 1ns/1ps
// Per-target labels follow frame-slot metadata through saved media and playback.
// Text occupies 64x16 pixels immediately above the upper-left corner; at the
// image top it moves inside the box, and at the right edge it shifts left.
module Shape_Labeled_Overlay #(parameter WIDTH=1280,HEIGHT=720,TARGETS=4,ID_ENABLE=0)(
    input wire enable,data_enable,metadata_valid,
    input wire [11:0] x,y,
    input wire [23:0] rgb,
    input wire [TARGETS*51-1:0] shapes,
    input wire [TARGETS*4-1:0] digits,
    input wire [TARGETS*8-1:0] track_ids,
    output reg [23:0] output_rgb
);
    function [63:0] glyph;
        input [7:0] c;
        begin case(c)
            "A":glyph=64'h1824427e42424200;"C":glyph=64'h3c42404040423c00;
            "D":glyph=64'h7844424242447800;
            "B":glyph=64'h7c42427c42427c00;"F":glyph=64'h7e40407c40404000;
            "#":glyph=64'h24247e247e242400;
            "E":glyph=64'h7e40407c40407e00;"G":glyph=64'h3c42404e42423c00;
            "I":glyph=64'h3c18181818183c00;"L":glyph=64'h4040404040407e00;
            "N":glyph=64'h4262524a46424200;"R":glyph=64'h7c42427c48444200;
            "T":glyph=64'h7e18181818181800;"/":glyph=64'h0204081020400000;
            "0":glyph=64'h3c464a5262463c00;"1":glyph=64'h1838181818187e00;
            "2":glyph=64'h3c42020c30407e00;"3":glyph=64'h3c42021c02423c00;
            "4":glyph=64'h0c1424447e040400;"5":glyph=64'h7e407c0202423c00;
            "6":glyph=64'h1c20407c42423c00;"7":glyph=64'h7e02040810101000;
            "8":glyph=64'h3c42423c42423c00;"9":glyph=64'h3c42423e02043800;
            default:glyph=0;
        endcase end
    endfunction
    function [7:0] hexchar;
        input [3:0] n;
        begin hexchar=n<10?8'd48+n:8'd55+n;end
    endfunction
    function [23:0] color;
        input [2:0] kind;
        begin color=kind==1?24'h00ff00:kind==2?24'h00ffff:
            kind==3?24'hffff00:24'hff80ff;end
    endfunction
    integer i;
    reg [50:0] box;
    reg [3:0] digit;
    reg [11:0] l,t,r,b,tx,ty,dx,dy;
    reg [63:0] label_text,font;
    reg [7:0] character;
    reg label_active;
    reg [23:0] label_color;
    reg [2:0] gx,gy;
    always @(*) begin
        output_rgb=rgb;box=0;digit=0;l=0;t=0;r=0;b=0;tx=0;ty=0;dx=0;dy=0;
        label_text=0;font=0;character=0;label_active=0;label_color=0;gx=0;gy=0;
        if(enable && data_enable && metadata_valid) begin
            for(i=TARGETS-1;i>=0;i=i-1) begin
                box=shapes[i*51 +: 51];l=box[47:36];t=box[35:24];r=box[23:12];b=box[11:0];
                if(box[50:48]!=0 && x>=l && x<=r && y>=t && y<=b &&
                   (x-l<2 || r-x<2 || y-t<2 || b-y<2)) output_rgb=color(box[50:48]);
            end
            // A label takes priority over intersecting borders; lower rank
            // loses only where two label areas themselves overlap.
            for(i=TARGETS-1;i>=0;i=i-1) begin
                box=shapes[i*51 +: 51];l=box[47:36];t=box[35:24];
                digit=digits[i*4 +: 4];
                tx=l>WIDTH-64?WIDTH-64:l;ty=t>=16?t-16:t;
                dx=x-tx;dy=y-ty;
                if(box[50:48]!=0 && x>=tx && x<tx+64 && y>=ty && y<ty+16) begin
                    label_text=box[50:48]==1?"RECT    ":box[50:48]==2?"CIRC/ELL":
                        box[50:48]==3?"TRIANGLE":"DIGIT 0 ";
                    if(box[50:48]==4 && digit<=9) label_text[15:8]=8'd48+digit;
                    if(ID_ENABLE) begin
                        // Full 8-bit ID in hexadecimal; fixed eight characters
                        // keep the existing label footprint and font datapath.
                        label_text=box[50:48]==1?"#00 RECT":box[50:48]==2?"#00 CIRC":
                            box[50:48]==3?"#00 TRI ":"#00 0   ";
                        label_text[55:48]=hexchar(track_ids[i*8+4 +: 4]);
                        label_text[47:40]=hexchar(track_ids[i*8 +: 4]);
                        if(box[50:48]==4) label_text[31:24]=8'd48+digit;
                    end
                    character=label_text[(7-dx[5:3])*8 +: 8];gx=dx[2:0];gy=dy[3:1];
                    label_color=color(box[50:48]);label_active=1;
                end
            end
            font=glyph(character);
            if(label_active) output_rgb=font[63-gy*8-gx]?label_color:24'h101820;
        end
    end
endmodule

// Zero-cycle overlay ahead of the existing registered OSD. Metadata is already
// latched for the selected display frame, including saved media frames.
module Shape_Overlay #(
    parameter integer WIDTH=1280,HEIGHT=720,TARGETS=1
)(
    input wire enable,data_enable,metadata_valid,
    input wire [11:0] x,y,
    input wire [23:0] rgb,
    input wire [49:0] shape,
    input wire [49:0] shape_second,
    output wire [23:0] output_rgb
);
    wire [1:0] kind=shape[49:48];
    wire [11:0] left=shape[47:36],top=shape[35:24],right=shape[23:12],bottom=shape[11:0];
    wire box=kind!=0 && x>=left && x<=right && y>=top && y<=bottom &&
        (x==left || x==right || y==top || y==bottom);
    wire [1:0] kind_second=shape_second[49:48];
    wire [11:0] left2=shape_second[47:36],top2=shape_second[35:24];
    wire [11:0] right2=shape_second[23:12],bottom2=shape_second[11:0];
    wire box_second=TARGETS==2 && kind_second!=0 && x>=left2 && x<=right2 && y>=top2 && y<=bottom2 &&
        (x==left2 || x==right2 || y==top2 || y==bottom2);
    // First-ranked target wins only at intersecting box pixels.
    assign output_rgb=(!enable || !data_enable)?rgb:
        (metadata_valid && box)?(kind==1?24'h00ff00:24'h00ffff):
        (metadata_valid && box_second)?(kind_second==1?24'h00ff00:24'h00ffff):rgb;
endmodule
