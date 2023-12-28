SET(SOURCES screens/ui_Index.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    images/ui_img_1756937144.c
    images/ui_img_330755264.c
    images/ui_img_temperatures_png.c
    images/ui_img_dust_png.c
    images/ui_img_1821463755.c
    fonts/ui_font_Kanit24.c
    fonts/ui_font_Segment42.c
    fonts/ui_font_Segment32.c)

add_library(ui ${SOURCES})
