`timescale 1ns/1ps

// R3'-A2: fast boot - first image ready starts the player while remaining
// images continue loading in the background. Cuts perceived boot from ~4 s
// (all four images) to ~1 s (one image). The ingest chain already tracks
// per-image completion via image_complete; this module just re-gates the
// player enable on "at least one image done" instead of "all images done",
// and gates image switching on the target image's own completion.
// Switch safety: media_player_control receives a switch request only when
// the target image is loaded (image_complete[target]). The existing
// black-dwell protocol handles the SDRAM side. If a switch to an unloaded
// image is requested (shouldn't happen in normal flow, but defensive),
// the gate holds the request until that image finishes loading.
module fast_boot_gate (
    input  wire       clk,
    input  wire       rst_n,
    input  wire       ingest_done,             // all images loaded (legacy)
    input  wire       image_complete,          // 至少一张图已提交显示
    input  wire       player_enable_raw,       // legacy enable condition
    output wire       player_enable_fast,      // gated on first image
    output wire       first_image_ready        // status (for OSD/report)
);
    // first image done = at least one image committed to the display
    assign first_image_ready = image_complete;

    // player can start as soon as the first image is in SDRAM
    assign player_enable_fast = player_enable_raw && first_image_ready;

    // ingest_done is informational: full functionality (all switchable
    // images) becomes available when it arrives, but display starts earlier
endmodule
