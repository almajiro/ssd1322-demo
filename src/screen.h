#define SSD1322_SETCOMMANDLOCK 0xFD
#define SSD1322_DISPLAYOFF 0xAE
#define SSD1322_DISPLAYON 0xAF
#define SSD1322_SETCLOCKDIVIDER 0xB3
#define SSD1322_SETDISPLAYOFFSET 0xA2
#define SSD1322_SETSTARTLINE 0xA1
#define SSD1322_SETREMAP 0xA0
#define SSD1322_FUNCTIONSEL 0xAB
#define SSD1322_DISPLAYENHANCE 0xB4
#define SSD1322_SETCONTRASTCURRENT 0xC1
#define SSD1322_MASTERCURRENTCONTROL 0xC7
#define SSD1322_SETPHASELENGTH 0xB1
#define SSD1322_DISPLAYENHANCEB 0xD1
#define SSD1322_SETPRECHARGEVOLTAGE 0xBB
#define SSD1322_SETSECONDPRECHARGEPERIOD 0xB6
#define SSD1322_SETVCOMH 0xBE
#define SSD1322_NORMALDISPLAY 0xA6
#define SSD1322_INVERSEDISPLAY 0xA7
#define SSD1322_SETMUXRATIO 0xCA
#define SSD1322_SETCOLUMNADDR 0x15
#define SSD1322_SETROWADDR 0x75
#define SSD1322_WRITERAM 0x5C
#define SSD1322_ENTIREDISPLAYON 0xA5
#define SSD1322_ENTIREDISPLAYOFF 0xA4
#define SSD1322_SETGPIO 0xB5
#define SSD1322_EXITPARTIALDISPLAY 0xA9
#define SSD1322_SELECTDEFAULTGRAYSCALE 0xB9

#define MIN_SEG	0x1C
#define MAX_SEG	0x5B


uint8_t screen1[8192] = {
17,21,97,17,17,17,18,34,34,34,34,86,138,187,204,206,221,221,221,221,220,186,152,100,35,115,67,34,34,34,34,33,34,34,19,82,18,34,35,52,105,164,68,68,69,68,107,122,152,105,68,51,53,151,82,18,34,39,167,123,218,172,186,153,170,171,187,153,168,154,190,117,69,103,130,18,17,107,65,34,51,68,68,68,68,71,197,116,69,68,68,68,67,50,130,34,34,34,39,34,18,34,51,39,98,35,34,51,53,120,154,187,188,221,221,221,221,205,204,169,116,51,51,34,34,39,202,17, 
17,22,49,17,17,17,18,34,34,34,33,55,153,171,188,221,221,221,221,220,187,170,152,100,53,82,52,66,34,34,33,18,18,17,36,50,17,17,35,69,137,166,68,68,68,68,124,138,151,168,68,68,51,151,66,34,18,41,183,124,202,118,104,155,186,117,106,186,168,203,188,117,68,86,146,17,23,220,82,34,52,68,68,68,68,86,197,132,68,85,68,67,67,50,99,17,50,34,39,33,34,35,34,39,115,50,35,51,69,120,170,187,188,221,221,221,205,204,203,169,116,51,51,34,34,39,189,97, 
17,54,17,17,17,17,17,34,18,34,34,38,170,171,188,221,221,221,221,204,187,186,152,100,53,50,34,68,50,34,34,34,17,17,22,50,17,18,34,34,88,115,68,67,68,68,124,155,136,182,68,68,51,150,49,34,34,58,188,236,134,155,221,220,220,221,183,123,203,203,206,101,85,70,130,33,124,187,85,35,68,68,68,68,68,86,196,132,68,67,68,67,51,34,86,33,18,34,39,66,34,50,34,56,195,51,51,51,69,121,170,171,188,221,221,221,221,204,218,153,116,51,51,34,51,54,205,194, 
17,100,17,33,17,17,17,17,18,33,35,37,155,187,188,221,221,221,221,204,187,170,152,117,56,33,34,34,68,50,34,33,33,17,22,17,17,34,34,35,51,52,68,68,68,68,91,187,138,132,68,68,51,117,34,34,34,56,206,182,140,187,186,189,205,188,221,184,171,136,154,101,85,87,146,24,201,186,71,67,68,67,68,68,68,69,196,132,68,68,67,69,50,34,86,17,17,34,38,98,38,130,34,36,164,51,51,51,70,121,154,171,188,221,221,221,220,220,203,153,116,51,34,34,34,54,205,217, 
18,98,17,18,17,17,17,17,18,34,34,53,138,186,188,221,221,221,221,204,187,170,152,117,54,34,34,34,34,52,50,33,17,16,38,18,34,17,18,34,35,51,68,68,68,68,74,186,138,100,68,68,68,102,34,33,34,88,152,89,186,187,186,203,204,221,189,203,105,171,136,153,184,86,146,125,169,185,86,116,68,68,52,67,68,68,198,131,69,67,68,73,51,34,54,18,34,35,36,81,21,97,34,36,181,35,50,51,70,121,154,171,188,205,221,221,205,219,187,168,100,51,34,34,50,54,188,220, 
19,97,33,17,17,17,17,17,17,34,34,52,137,187,189,204,221,221,221,203,187,170,153,118,117,34,34,34,34,34,35,51,50,34,69,33,17,17,18,34,35,51,52,68,68,67,87,186,152,68,51,68,68,102,50,34,35,153,150,155,156,170,187,151,86,104,172,201,188,236,220,171,199,103,148,219,169,199,86,149,68,68,68,68,68,68,184,132,67,67,52,86,34,33,39,17,17,17,36,114,33,34,34,35,135,51,51,52,86,137,154,171,188,205,221,221,221,203,172,168,100,51,34,34,34,53,204,220, 
39,129,17,33,17,17,17,17,18,34,34,52,137,172,189,204,221,221,220,203,187,170,153,118,115,35,34,50,34,34,34,18,132,17,65,17,17,17,34,17,34,51,52,67,67,67,69,169,150,68,68,68,68,101,68,51,39,169,119,167,138,168,68,102,102,102,101,122,188,219,174,202,183,104,170,187,170,198,102,120,68,68,68,68,67,68,169,134,67,68,51,69,33,34,24,17,17,17,19,130,17,18,18,51,136,50,51,52,86,137,154,171,188,205,221,221,221,203,171,152,100,51,34,34,34,53,188,187, 
74,97,17,17,17,17,17,17,34,34,34,37,137,170,204,204,221,221,221,203,187,170,169,118,131,50,34,34,34,34,33,37,150,17,97,17,17,17,17,18,34,51,51,52,51,68,52,168,133,68,68,52,68,153,68,68,73,169,73,152,184,69,85,102,103,118,119,134,124,219,173,199,151,120,189,188,203,168,103,121,84,68,68,52,68,67,170,118,51,51,51,34,34,17,22,17,17,1,18,130,34,34,34,35,108,115,51,52,103,137,154,170,188,204,221,221,220,187,171,168,99,51,34,34,34,52,187,170, 
75,33,17,17,17,17,17,17,18,34,35,59,236,171,204,204,205,221,220,204,187,170,169,134,115,35,34,34,34,34,33,22,116,18,81,17,17,17,33,33,34,34,51,51,67,67,67,167,100,68,68,68,72,204,166,84,138,166,104,136,115,69,85,102,119,134,136,136,91,187,220,184,136,155,222,221,218,121,119,120,132,68,68,68,52,67,154,102,51,51,50,34,34,18,22,49,17,16,16,84,34,34,34,35,124,180,51,52,103,153,154,170,187,204,221,221,204,186,170,184,83,51,34,34,34,36,140,203, 
103,17,17,17,17,17,17,18,34,34,52,190,235,170,203,204,204,221,205,204,187,170,153,137,115,50,34,34,34,33,17,20,82,18,80,17,17,17,17,18,18,34,35,51,52,51,51,156,148,68,84,84,123,168,155,134,170,131,120,104,67,53,87,120,137,136,89,136,125,172,217,155,153,205,222,238,218,136,151,121,166,67,68,68,139,116,139,88,51,51,34,34,17,34,22,82,17,16,18,85,34,34,34,35,124,215,51,52,103,153,170,170,187,204,221,221,221,186,169,184,83,50,50,34,34,35,73,204, 
82,17,1,17,17,17,17,18,34,34,58,205,169,171,187,188,204,221,220,204,187,186,169,138,100,50,35,34,34,33,17,18,48,20,64,17,17,18,33,17,18,34,35,51,51,51,52,136,222,165,101,85,187,152,154,188,203,83,87,132,67,84,103,120,119,134,87,118,122,203,185,121,171,206,238,238,186,169,152,138,217,68,68,157,232,68,107,72,51,51,34,34,33,33,19,97,17,16,1,88,34,34,34,35,107,204,67,68,103,153,170,171,188,204,221,221,222,203,169,184,83,50,50,34,33,34,52,139, 
17,17,17,1,17,17,17,18,34,35,76,201,104,156,170,187,221,205,220,204,187,170,169,137,84,50,34,34,34,33,17,17,17,5,17,17,17,18,33,17,17,34,34,51,51,51,51,120,174,237,85,85,187,185,171,222,219,67,71,85,85,69,86,102,102,101,149,101,100,138,86,104,156,238,238,236,187,186,170,188,186,101,93,237,152,131,107,72,51,51,34,17,33,18,34,97,16,17,0,39,18,34,34,51,108,205,116,68,103,153,170,171,187,204,221,221,221,202,153,169,83,51,50,34,34,18,35,71, 
17,17,17,17,17,17,17,34,18,35,102,83,104,172,170,171,204,221,204,204,187,170,169,153,84,51,50,34,34,17,17,17,17,22,17,18,17,17,17,17,17,34,34,51,51,51,53,136,155,222,134,204,206,187,221,237,217,68,119,69,85,69,52,69,84,84,137,68,53,102,102,86,171,238,238,220,204,203,203,220,205,181,157,200,133,164,105,105,51,34,34,33,17,17,18,83,17,0,0,6,17,34,34,51,107,186,147,69,104,153,170,187,188,205,220,205,221,185,169,155,67,51,50,34,34,17,18,34, 
17,17,17,17,17,17,18,34,34,35,35,51,88,171,154,171,188,221,220,204,187,186,169,168,101,51,50,34,33,17,17,17,17,5,17,17,17,17,17,17,17,17,34,51,51,51,59,167,169,205,202,219,238,238,238,238,229,67,87,85,86,84,67,68,69,68,156,133,67,70,84,116,104,206,238,238,238,222,238,238,204,215,204,135,147,120,89,121,50,34,34,17,33,17,21,104,33,1,16,6,33,18,35,51,73,187,132,69,120,153,170,187,188,221,204,204,203,169,153,138,83,51,34,34,33,18,17,33, 
17,17,17,17,17,18,34,34,34,34,34,34,72,170,154,171,187,205,220,204,187,186,169,184,101,51,34,34,33,17,17,17,17,68,16,17,17,17,17,17,17,17,35,34,34,51,156,168,170,172,235,185,238,238,238,238,212,68,85,85,86,116,52,52,68,67,173,183,67,69,85,53,136,141,238,238,238,238,238,238,219,218,218,153,115,89,104,120,66,34,33,18,17,18,21,103,32,16,1,24,65,17,35,51,54,171,116,85,120,153,170,187,204,204,204,187,170,154,153,136,99,51,34,34,34,33,17,17, 
17,17,17,17,17,34,34,34,34,33,34,34,72,201,154,170,187,204,221,204,203,186,169,200,117,50,34,34,17,17,17,17,17,84,17,17,17,17,17,17,17,18,34,34,35,38,203,152,156,172,220,222,238,236,204,205,165,69,116,84,70,148,68,67,68,67,172,203,116,36,86,36,87,72,190,220,220,221,206,238,237,205,187,133,67,55,135,119,82,33,17,35,18,17,19,101,17,1,17,5,97,17,35,51,52,106,101,86,120,153,154,187,204,204,204,187,170,153,152,135,114,34,34,34,34,34,33,17, 
17,17,17,17,18,34,34,33,17,34,34,34,74,185,153,170,187,204,204,204,187,186,188,200,101,66,34,33,17,17,17,17,17,81,17,17,17,18,17,17,17,34,33,34,34,59,168,137,172,204,205,238,238,188,204,205,149,52,116,68,71,216,99,51,68,54,132,3,103,35,69,67,67,68,91,204,220,221,204,238,238,220,203,165,67,53,136,135,98,33,34,34,17,17,17,17,17,0,16,19,96,17,18,51,52,69,85,86,120,153,170,188,188,204,203,187,170,153,152,117,115,34,34,34,34,33,17,17, 
17,17,17,17,34,34,17,17,17,33,34,34,74,153,153,154,187,204,204,204,203,170,189,200,101,51,34,17,17,17,17,17,33,113,17,17,16,17,17,33,18,33,18,34,35,140,168,137,158,170,222,238,220,171,203,188,117,52,99,68,73,237,120,51,52,57,134,52,118,162,68,98,52,52,69,139,204,205,204,206,238,237,169,203,67,51,120,133,98,33,18,33,17,17,17,17,0,0,16,2,113,1,17,51,68,68,68,87,136,153,171,187,187,204,203,187,169,153,136,117,116,34,34,34,34,17,17,17, 
17,17,17,34,33,17,17,17,17,17,34,34,90,137,137,170,187,188,204,204,188,186,172,168,101,67,34,17,17,17,17,17,18,81,17,17,17,17,17,17,34,17,17,34,35,172,168,137,158,189,238,237,188,186,203,187,100,52,83,51,106,218,169,181,34,140,216,117,139,148,52,84,36,67,88,167,155,204,188,220,238,238,202,202,67,35,103,148,114,17,17,33,17,17,17,17,17,17,17,18,130,17,17,19,52,68,68,87,136,153,170,186,187,188,203,186,169,153,152,117,102,50,34,34,17,17,17,17, 
17,17,17,17,17,17,17,17,18,34,34,34,122,137,137,170,171,187,204,187,187,187,169,136,117,66,17,17,17,17,17,17,20,49,17,17,17,17,17,18,33,17,17,18,54,204,169,137,154,238,238,237,187,171,218,188,68,51,67,51,122,98,73,204,168,190,234,201,110,199,99,53,35,51,69,155,167,172,205,221,221,238,236,182,51,34,71,163,131,33,33,33,17,17,17,17,0,0,17,1,115,17,17,17,52,68,69,103,137,154,170,170,187,187,187,170,169,153,136,117,55,34,34,34,17,17,17,17, 
17,17,17,17,17,17,17,17,18,34,34,34,135,137,153,170,171,187,187,187,187,186,185,136,101,66,17,17,17,17,17,17,22,50,17,17,16,17,2,34,17,17,17,18,41,204,186,153,189,238,237,220,187,171,202,187,68,51,67,51,116,66,77,221,222,222,238,187,189,216,150,69,51,52,53,119,203,121,204,221,221,222,238,202,99,35,53,164,163,50,34,18,17,17,17,17,0,0,1,1,84,20,117,33,35,68,69,103,137,170,170,171,187,187,187,170,153,153,136,116,55,50,34,33,17,17,17,17, 
17,17,17,17,17,17,17,17,18,34,34,36,134,137,153,154,170,187,187,187,187,170,153,151,101,50,17,17,17,33,17,34,39,34,34,33,17,2,50,68,17,33,17,17,58,172,138,186,170,206,187,203,171,170,171,219,84,51,68,68,120,56,91,238,237,238,221,221,220,216,201,68,50,50,52,72,90,169,156,221,221,222,237,169,115,50,52,166,168,50,52,34,33,17,17,17,17,17,17,17,84,22,134,33,34,52,85,104,137,153,170,171,187,187,186,170,153,153,136,116,54,82,34,34,17,17,17,17, 
17,17,17,17,17,17,17,18,34,34,34,56,133,137,137,153,170,171,187,187,187,170,152,136,134,67,33,17,17,33,17,18,72,52,51,33,18,34,18,114,17,17,17,18,72,187,117,150,123,221,187,186,187,171,171,219,67,50,68,71,139,139,171,222,236,222,237,221,221,214,200,36,66,68,51,69,117,171,138,221,221,221,236,202,132,51,69,170,151,51,56,165,34,17,17,17,17,17,17,17,88,18,34,34,34,35,69,120,137,153,170,170,187,187,170,169,153,136,136,100,52,114,34,34,17,17,17,17, 
17,17,17,17,17,17,17,34,34,34,35,75,133,120,153,153,154,170,187,187,186,169,136,119,101,84,50,17,17,17,18,18,105,170,151,67,50,20,55,81,17,17,17,17,90,202,102,134,140,221,187,186,186,171,170,202,83,34,68,70,187,203,188,222,238,238,237,221,221,198,180,36,66,41,82,69,117,120,183,204,205,205,237,218,166,34,69,155,132,51,52,121,66,17,17,17,17,17,17,17,55,18,17,33,34,34,70,119,137,153,170,170,187,186,170,153,136,136,136,100,51,131,50,34,33,17,17,17, 
17,17,17,17,17,17,18,34,34,34,51,89,69,120,136,137,153,170,187,187,170,153,136,118,85,50,51,67,50,33,33,18,120,187,187,182,87,133,86,33,17,17,18,18,91,202,101,149,156,236,187,186,187,155,186,184,50,50,68,84,189,221,221,221,238,238,237,222,222,168,178,20,66,37,84,101,102,106,121,172,222,221,237,218,153,51,68,138,114,35,67,88,84,49,33,17,18,17,17,17,22,17,33,33,18,52,51,103,137,153,170,171,187,186,170,153,136,136,135,100,51,133,50,34,34,17,17,17, 
17,17,17,17,17,17,34,34,34,34,35,86,52,104,136,136,153,170,187,186,170,153,135,101,84,52,34,133,34,51,35,51,135,139,204,202,117,52,148,17,17,17,18,17,123,201,101,133,173,236,171,171,236,171,170,214,34,50,52,68,109,221,221,221,237,237,238,222,236,213,81,34,67,34,52,118,103,41,185,124,238,221,238,218,154,115,68,121,82,51,69,72,101,131,34,35,67,34,17,33,23,65,18,35,68,34,51,87,136,153,154,170,170,170,169,153,136,136,135,100,51,87,50,34,34,17,17,17, 
17,17,17,17,17,34,34,34,34,35,51,134,52,103,120,136,153,170,170,170,153,152,118,101,84,67,36,100,33,17,17,18,118,107,202,150,68,87,68,1,17,17,17,17,122,200,102,117,172,236,187,170,219,170,154,180,34,50,36,68,39,221,221,222,222,238,237,237,237,194,17,18,68,34,36,36,87,51,155,153,222,205,222,218,169,165,52,105,83,121,136,102,116,105,199,119,119,134,50,18,22,99,51,34,23,98,52,69,120,137,154,170,170,170,169,152,136,119,118,83,51,56,51,34,34,33,17,17, 
17,17,17,18,34,34,34,34,34,34,38,179,52,87,120,136,153,170,170,153,153,135,118,101,84,51,35,81,33,34,33,52,101,107,200,66,71,119,97,17,33,17,17,17,120,199,102,101,190,234,173,153,203,155,170,117,50,50,36,68,66,124,222,222,237,237,205,222,222,146,17,17,52,33,34,51,87,82,74,188,157,204,238,220,153,169,67,104,102,86,138,150,101,238,202,151,103,116,88,83,54,114,34,34,37,98,51,69,103,137,153,153,154,170,169,152,136,119,118,67,51,54,83,34,34,34,17,17, 
17,17,18,34,34,34,34,34,34,34,38,147,51,86,120,137,153,153,153,153,152,119,102,85,84,67,34,17,17,18,69,100,118,157,164,120,84,66,17,17,50,17,17,17,119,183,86,101,206,234,171,203,186,188,170,88,50,50,19,68,67,106,188,238,237,221,221,237,236,83,98,18,68,34,34,50,103,116,37,204,203,204,222,236,186,154,99,88,97,20,152,103,71,237,206,237,103,103,136,180,36,129,34,34,35,82,52,69,86,120,153,153,170,153,153,152,136,119,118,67,51,53,163,50,34,34,33,17, 
17,18,34,34,34,34,34,34,34,34,40,98,35,86,120,136,136,137,153,152,135,118,102,85,85,67,50,34,35,87,135,71,101,154,104,187,34,50,17,17,33,34,34,33,119,166,86,86,222,218,189,189,185,205,184,104,50,50,18,51,68,70,222,221,222,236,236,155,164,35,81,17,52,34,50,34,103,121,83,172,204,188,238,238,236,169,148,72,97,70,103,84,137,202,189,206,151,119,136,155,51,98,34,34,34,34,52,68,85,103,137,153,153,153,153,153,136,119,117,67,35,53,200,51,34,34,34,17, 
17,34,34,34,34,34,34,34,34,34,72,34,35,86,120,136,136,136,136,136,119,102,102,85,85,67,50,34,72,171,203,118,133,137,120,121,52,67,17,17,18,67,51,34,120,150,102,87,222,217,190,201,185,204,167,137,66,35,18,51,52,68,91,238,238,222,196,56,49,33,18,34,53,50,37,50,86,121,148,92,188,189,238,238,238,216,167,72,102,102,102,115,206,169,202,205,135,104,119,138,147,146,34,18,34,35,52,68,85,102,120,153,153,153,136,136,136,119,117,67,34,54,172,67,34,34,34,17, 
17,34,34,34,34,34,34,34,34,33,86,34,51,86,119,120,120,136,136,119,118,102,101,85,68,67,50,37,172,204,205,200,121,87,52,150,35,67,17,18,35,116,68,50,105,149,102,88,222,202,171,153,170,170,150,152,98,35,34,35,51,68,68,139,205,238,237,215,33,17,34,34,53,50,52,131,69,105,153,56,172,171,222,238,238,217,137,89,135,102,101,70,238,169,138,220,150,136,120,121,181,134,34,34,34,35,52,69,85,86,103,156,219,152,136,135,136,119,118,67,51,53,156,82,33,17,17,17, 
17,17,17,17,17,34,34,34,33,17,152,35,52,70,103,119,119,119,119,103,102,102,85,85,84,67,51,90,172,204,205,221,168,102,34,66,68,65,17,34,53,152,135,100,121,133,102,88,222,185,169,153,168,169,135,152,131,50,50,53,36,36,68,70,155,187,237,167,17,33,34,34,52,50,52,134,52,89,153,133,171,171,206,238,238,233,136,168,119,118,67,53,222,200,121,221,200,136,188,121,154,118,34,34,34,51,68,68,85,102,106,221,235,153,136,135,119,119,117,67,51,52,122,66,17,17,17,17, 
17,18,17,17,34,33,17,17,17,19,132,51,51,69,102,102,119,118,102,102,102,101,85,85,68,67,52,186,188,172,237,238,234,202,86,102,69,50,17,34,70,187,203,101,121,118,102,89,206,168,153,137,152,153,136,137,183,50,39,155,237,168,67,69,121,134,170,169,33,35,34,34,52,34,35,105,68,73,152,165,154,170,206,238,218,120,138,152,135,118,36,101,89,219,190,221,119,121,237,137,171,120,34,34,34,51,52,68,85,85,123,187,201,136,136,136,103,102,101,68,51,51,86,50,17,17,17,17, 
17,34,17,17,17,17,17,17,18,37,115,51,51,69,86,102,102,102,102,102,102,85,85,84,84,67,58,187,188,186,238,222,236,172,168,103,152,33,18,51,73,173,236,134,121,118,102,90,206,168,153,137,152,136,121,137,202,51,142,238,238,220,168,52,103,101,154,186,34,34,98,35,52,51,51,73,83,71,152,153,154,170,190,237,117,71,187,170,152,134,52,70,74,238,188,234,103,141,238,168,170,150,50,34,34,51,68,68,69,86,139,186,119,136,136,136,118,102,84,67,51,51,51,34,17,17,17,17, 
17,33,17,17,17,17,17,17,18,56,99,35,51,52,85,86,101,86,102,101,85,85,85,84,68,51,123,171,205,185,189,220,220,186,184,172,202,66,34,52,91,173,219,134,122,118,102,91,205,152,152,137,136,152,121,136,169,53,238,238,238,235,188,220,117,103,171,154,101,51,72,34,35,51,51,72,67,70,170,137,138,153,190,237,117,69,185,101,86,83,35,86,53,221,151,84,105,222,237,214,138,154,34,34,34,51,68,68,84,85,103,118,86,103,136,135,119,102,84,67,51,52,51,34,33,17,17,17, 
17,17,17,17,17,17,18,34,50,56,67,50,34,52,69,85,85,85,85,85,85,85,85,68,68,52,171,170,188,205,172,204,204,202,203,187,186,115,52,69,91,154,170,102,137,118,102,107,206,168,151,138,152,153,123,152,168,90,238,238,238,219,201,139,217,70,222,238,238,216,57,98,51,52,52,71,52,71,203,169,139,185,189,220,117,67,134,86,117,50,51,100,67,85,68,75,188,205,238,231,104,152,34,34,34,51,68,68,84,85,85,101,102,86,103,136,119,101,101,67,51,51,50,34,33,17,17,17, 
17,17,17,17,17,18,34,34,35,139,67,50,34,35,68,68,68,85,85,85,85,85,84,68,67,69,172,139,188,206,203,204,171,201,202,187,186,115,55,137,139,102,102,87,137,102,102,108,206,167,151,140,153,186,126,152,168,125,238,237,221,220,222,221,190,180,158,222,237,238,201,147,51,52,99,70,39,118,203,167,157,217,189,220,133,67,102,119,119,67,51,52,68,102,68,34,156,222,237,217,118,135,98,50,34,51,68,68,85,85,85,85,85,101,85,104,135,119,100,67,52,120,115,34,33,17,17,17, 
17,17,17,17,34,34,51,51,53,186,67,34,34,35,52,68,68,85,85,85,85,85,84,68,67,52,188,172,187,190,237,170,202,187,171,153,153,132,72,188,217,101,67,55,153,102,118,124,206,151,151,140,138,219,141,152,169,174,238,238,186,186,170,151,190,219,77,237,238,222,237,150,67,52,117,68,58,148,136,101,155,201,173,220,117,52,72,120,136,50,35,51,149,87,92,116,76,238,237,168,102,134,130,34,35,51,68,68,69,68,85,85,85,85,85,69,137,153,85,68,71,170,131,34,34,17,17,17, 
17,17,17,34,34,51,51,51,73,201,50,34,34,35,51,52,68,69,101,85,84,69,68,68,68,68,171,190,187,188,219,237,187,155,187,136,136,131,88,222,231,100,51,89,153,102,118,139,220,135,152,137,137,186,137,137,152,189,222,238,238,238,189,203,236,222,123,238,221,221,238,204,135,68,103,67,122,150,68,73,153,169,173,203,117,68,72,137,153,66,51,56,185,85,141,184,172,238,219,187,85,102,114,34,51,51,68,68,68,69,85,85,86,85,85,84,91,198,84,68,106,168,99,50,34,17,17,17, 
17,18,34,51,51,51,52,52,123,200,50,34,33,18,35,51,68,68,68,69,85,85,68,68,67,52,170,206,217,188,205,222,219,153,187,119,135,115,103,206,214,100,72,73,152,102,117,155,220,152,152,121,136,152,152,137,153,172,237,222,238,238,189,221,167,222,168,221,203,204,222,202,189,132,105,68,139,151,121,171,169,168,172,203,133,69,87,154,169,67,35,41,168,102,157,221,222,238,212,40,85,83,82,34,51,51,52,68,68,68,84,85,85,85,84,84,121,135,84,69,154,166,100,34,34,34,17,17, 
18,35,51,68,68,68,52,70,155,215,50,35,17,18,35,51,68,68,68,68,86,101,84,51,51,52,155,188,204,171,237,216,53,120,171,156,167,115,91,154,197,70,152,122,151,86,118,155,219,152,152,137,120,152,153,137,172,220,205,238,238,219,189,218,85,236,148,169,101,139,189,188,205,236,168,70,154,152,121,173,185,169,156,203,132,69,87,154,187,69,51,40,168,132,141,221,238,238,213,133,84,66,34,34,51,51,52,68,68,68,85,69,119,117,84,70,115,70,84,72,170,166,101,50,34,17,17,17, 
34,35,52,86,101,85,68,86,171,213,34,49,17,17,35,51,51,68,68,68,124,216,84,67,51,52,138,206,221,186,205,149,29,121,138,186,199,98,58,121,165,137,75,153,151,102,102,171,235,154,168,153,136,136,153,153,220,205,238,238,220,204,219,154,155,214,98,116,68,70,172,189,206,238,238,204,203,169,136,155,169,168,172,218,132,85,103,171,188,134,67,51,169,133,140,221,222,238,236,203,84,66,34,34,35,51,52,68,68,68,84,68,107,132,68,86,68,51,84,106,170,150,118,51,34,33,17,17, 
34,35,69,154,151,119,101,87,189,131,52,17,17,17,34,51,51,68,68,68,190,238,101,67,51,52,122,205,221,236,172,147,61,186,186,187,182,114,38,202,136,131,106,137,150,86,118,171,234,189,202,153,137,152,137,137,189,238,238,237,237,220,171,202,141,151,115,104,67,54,104,155,205,238,222,222,219,205,185,155,137,154,172,218,132,85,103,171,203,149,84,66,73,104,94,221,222,237,237,216,99,66,34,34,34,51,52,68,68,68,68,69,118,101,69,84,67,51,52,170,154,135,119,51,50,34,34,17, 
34,51,70,190,219,152,119,103,185,133,113,17,17,17,35,51,51,51,51,68,157,238,132,51,51,51,107,187,121,222,220,235,205,219,185,156,164,99,37,188,170,134,137,74,150,102,102,172,218,190,234,153,139,200,153,156,238,238,238,238,219,169,156,186,203,72,102,118,148,68,102,136,157,237,238,237,238,185,203,141,168,170,189,234,149,86,103,156,203,117,84,84,34,90,77,238,238,238,221,195,83,50,34,34,34,51,51,52,68,68,68,69,69,69,84,68,52,51,57,185,155,135,136,51,34,34,34,33, 
34,51,69,173,238,235,168,119,151,172,49,17,17,18,35,51,51,51,51,51,107,203,148,51,51,51,73,134,98,189,237,238,221,219,185,187,115,70,121,172,185,154,152,57,149,102,102,188,220,173,202,169,139,168,153,190,238,238,238,238,237,185,171,187,197,70,154,152,106,67,103,86,188,222,238,238,222,234,139,155,152,170,157,220,180,86,120,172,202,117,68,68,66,42,123,237,221,222,237,148,67,34,34,34,35,51,51,51,68,68,68,68,68,68,68,68,67,52,107,186,170,119,137,66,34,34,34,34, 
34,34,69,173,238,238,218,119,86,167,114,33,17,34,34,51,51,51,51,51,54,137,84,51,51,51,55,135,167,221,238,221,221,219,186,170,66,34,36,185,105,123,148,90,149,86,153,186,205,188,154,185,154,136,170,238,238,238,238,238,218,153,187,171,100,52,121,185,168,163,70,86,171,189,237,237,237,238,140,154,153,185,205,188,198,102,119,188,183,101,68,69,83,57,168,238,237,222,220,52,34,34,18,34,35,51,51,51,68,68,68,68,68,68,68,68,68,53,187,186,169,119,137,82,34,34,34,34, 
34,34,52,172,222,238,237,150,104,68,84,17,17,18,34,51,51,51,51,51,51,68,51,51,51,51,53,151,205,221,238,237,221,219,183,165,34,33,40,151,72,186,131,90,134,85,153,153,172,235,153,185,154,152,173,238,238,238,238,238,169,137,170,186,69,52,70,120,219,137,53,102,72,187,205,238,238,237,170,169,170,171,219,138,186,118,120,173,151,101,84,67,84,107,186,190,238,236,197,51,34,34,34,34,34,51,51,51,52,68,67,52,68,68,68,68,68,71,202,186,184,119,137,97,17,18,34,34, 
17,33,35,156,222,238,238,200,116,67,35,49,17,34,35,52,51,51,51,35,51,51,51,51,50,50,39,135,173,221,238,221,237,219,167,132,34,34,72,71,104,185,98,106,134,86,103,122,171,206,154,202,154,153,190,238,238,238,238,234,137,171,187,168,51,68,52,69,152,86,83,86,86,123,187,205,237,237,169,153,187,205,201,135,101,103,135,188,119,84,117,52,69,87,138,172,237,146,51,50,34,34,34,34,99,51,51,51,68,68,67,68,68,69,69,68,68,88,186,186,183,120,136,113,17,17,17,34, 
17,34,35,123,205,238,238,234,101,102,50,34,18,34,51,68,68,51,50,35,51,51,51,51,51,50,88,120,125,222,221,205,237,235,149,50,18,34,115,39,90,169,66,106,135,102,86,139,202,189,219,234,153,189,222,238,238,238,238,184,139,187,169,148,33,51,67,51,84,67,84,36,101,101,156,187,205,220,151,135,122,237,153,133,69,103,136,201,118,84,68,52,70,68,69,171,189,217,66,34,34,34,34,39,85,35,51,51,51,51,68,68,68,84,85,84,68,89,203,187,166,120,136,129,16,0,17,17, 
17,17,18,73,205,238,238,237,134,155,50,50,34,35,68,85,84,67,50,34,35,51,51,51,67,51,135,104,123,221,236,206,222,216,115,34,34,36,66,70,123,166,50,105,151,102,87,171,188,187,221,200,139,204,238,238,238,238,235,152,120,187,170,131,18,35,52,68,53,101,69,53,53,101,88,187,187,169,152,137,156,235,169,116,102,103,136,182,101,68,51,51,86,34,37,155,206,238,82,34,34,34,35,136,137,67,51,51,52,67,68,68,69,86,118,101,69,106,203,171,150,136,120,146,17,0,16,0, 
17,17,17,37,189,222,238,238,183,204,50,34,34,35,86,119,118,84,50,34,34,35,51,51,51,38,133,103,136,206,221,235,140,152,66,49,34,34,34,86,171,148,50,122,151,86,90,203,172,188,205,201,154,206,238,238,238,222,202,153,148,121,185,65,18,20,52,52,52,69,51,67,68,133,71,104,154,152,139,154,190,187,169,68,102,103,136,150,101,68,52,51,85,68,74,186,238,238,146,34,34,34,72,136,136,83,51,51,51,51,52,68,70,152,169,150,85,107,203,171,135,135,120,148,33,17,16,0, 
17,17,17,35,124,222,238,238,235,237,83,34,34,52,105,170,168,100,50,34,34,34,34,34,34,72,101,87,120,137,222,216,120,169,98,34,33,33,34,104,186,115,51,122,151,103,140,203,187,187,205,236,156,238,238,238,238,204,188,170,198,53,101,34,34,19,52,68,84,52,100,68,68,84,90,167,69,135,138,187,237,203,165,69,102,103,137,101,84,67,51,52,69,82,138,172,238,238,195,34,34,36,136,119,102,100,51,51,51,51,68,68,86,156,204,167,85,107,203,171,136,119,120,133,33,17,17,17, 
17,17,17,34,57,221,238,238,238,237,131,34,35,52,106,222,202,100,51,34,34,34,34,34,35,134,84,87,136,135,120,188,170,186,152,34,34,34,34,106,185,83,51,138,152,120,205,187,187,187,205,221,190,238,238,238,238,187,222,238,235,34,37,50,34,35,67,51,35,51,52,103,86,84,189,186,133,69,93,238,236,187,117,69,102,119,137,85,84,67,51,51,85,35,171,174,238,238,203,130,50,89,136,118,87,116,51,51,51,52,68,68,86,157,237,167,85,108,187,171,135,119,120,134,34,34,17,17, 
17,17,17,34,52,189,238,238,238,237,163,50,35,52,105,222,217,100,51,34,34,34,34,34,38,102,85,136,136,152,84,73,125,187,168,50,34,34,34,139,183,67,67,138,152,123,221,187,187,202,206,235,206,238,238,238,189,156,221,220,220,82,37,51,35,35,84,51,51,68,67,51,67,68,205,237,204,201,90,205,221,185,84,85,102,119,120,69,68,52,51,68,83,52,186,205,238,238,202,219,83,119,135,117,70,116,51,51,51,51,68,68,70,156,221,167,85,108,188,170,135,119,120,135,34,34,34,17, 
18,17,18,35,51,91,222,238,238,221,179,50,51,52,106,204,201,100,51,34,34,34,34,34,72,101,102,188,202,166,121,153,74,169,166,51,34,34,34,138,166,68,52,138,168,174,235,187,188,203,237,187,189,238,238,237,171,189,220,204,205,147,35,66,50,51,117,69,72,215,102,83,67,53,220,204,204,204,185,170,206,206,181,86,102,119,119,85,68,51,51,52,66,36,170,206,238,238,218,157,184,136,135,101,70,133,51,51,52,51,52,68,69,153,170,166,85,108,188,170,135,119,136,136,34,34,34,34, 
18,34,34,34,51,54,205,238,238,188,211,35,35,52,87,120,136,84,50,50,34,34,34,34,119,101,91,188,169,173,218,204,185,169,83,101,67,50,35,154,165,67,140,155,169,222,219,186,187,202,219,155,187,189,238,238,155,188,203,203,204,164,51,66,35,36,118,102,104,200,102,101,69,72,204,204,204,203,186,169,140,190,236,117,102,119,118,84,68,51,52,52,50,37,139,206,238,238,221,120,171,135,119,101,69,149,67,51,51,52,68,52,69,86,102,102,84,91,171,185,135,119,136,136,50,50,34,34, 
34,34,34,34,35,52,122,238,238,139,199,50,51,51,69,85,86,67,50,34,34,34,34,37,118,66,124,204,171,140,156,235,134,149,70,154,138,165,36,154,149,52,172,171,156,238,187,171,188,201,168,155,187,186,189,238,185,153,136,136,136,149,50,82,51,39,136,86,101,102,86,102,102,88,153,153,153,152,119,118,105,157,238,202,86,103,117,68,68,67,68,68,50,36,139,206,238,238,238,167,105,201,119,85,85,134,67,51,68,52,52,68,68,69,85,85,68,107,155,184,136,136,137,153,50,50,34,34, 
34,34,34,35,50,51,70,138,220,138,200,51,51,51,52,68,68,51,51,51,35,34,34,71,116,51,139,190,237,154,155,134,133,99,56,139,238,236,69,137,132,68,119,170,189,236,186,187,204,185,152,138,187,221,203,189,230,136,135,120,119,135,67,66,35,122,154,134,101,118,118,102,187,104,153,152,152,135,118,102,119,140,221,221,182,103,116,68,68,68,52,71,50,35,138,222,238,238,222,185,152,173,167,85,68,149,84,68,68,52,68,68,68,68,68,68,68,138,154,184,136,136,153,153,50,34,34,34, 
34,50,34,52,51,51,70,120,168,136,202,51,51,51,51,67,67,51,51,51,51,34,50,102,68,51,156,222,222,218,118,87,67,120,120,222,238,238,118,153,117,68,68,171,206,219,186,187,203,153,152,136,186,188,238,221,217,120,119,118,103,119,98,50,55,152,168,167,102,71,102,102,121,120,136,136,136,119,118,104,135,139,205,221,219,118,100,68,68,68,84,153,50,35,122,238,238,238,221,169,154,170,217,101,68,134,69,68,68,68,68,68,68,68,68,68,68,137,153,168,153,152,135,102,51,51,35,35, 
35,51,51,52,51,51,70,103,135,119,187,51,51,51,51,51,51,51,51,51,51,51,51,117,67,36,153,237,222,200,100,136,151,86,142,237,222,238,186,136,101,68,85,172,221,203,171,188,203,169,152,153,171,171,222,238,233,119,103,118,102,102,82,51,103,119,151,152,151,118,104,102,103,136,136,136,136,119,118,121,170,187,187,188,204,183,84,68,85,85,85,170,50,50,74,238,238,238,204,117,135,135,206,134,84,149,87,68,68,68,68,68,68,68,68,68,68,121,137,152,119,102,102,102,51,51,51,50, 
51,51,51,52,51,52,101,87,103,119,187,84,51,51,51,67,51,51,51,51,51,51,36,101,67,37,139,222,237,182,118,186,182,71,186,206,238,238,203,104,100,68,85,172,220,187,171,187,186,169,152,135,155,169,190,236,183,136,136,136,136,136,67,51,68,135,121,150,102,134,87,119,102,153,153,153,153,152,135,154,203,171,170,187,187,186,67,69,102,151,101,171,50,34,42,238,237,238,185,51,67,51,189,232,117,134,86,84,68,68,68,68,68,68,68,68,68,121,138,134,102,101,86,102,52,51,51,51, 
51,52,68,68,68,69,85,86,120,119,171,115,67,52,51,51,51,51,51,51,51,51,53,101,66,54,166,188,219,102,105,155,165,108,222,238,238,238,185,102,84,85,70,188,219,186,187,187,186,170,152,135,87,169,138,186,155,137,170,153,153,153,53,34,34,87,136,154,118,86,101,85,119,170,170,170,169,153,153,171,188,186,169,170,170,167,68,69,142,238,101,155,66,34,43,238,222,238,148,34,51,68,188,221,90,135,71,100,85,85,84,68,68,68,68,68,85,120,154,118,102,85,86,103,68,68,67,51, 
52,68,85,85,68,71,85,102,120,136,155,148,68,68,68,68,68,52,67,51,51,51,52,100,51,69,151,157,198,102,152,138,168,238,238,238,238,237,182,102,69,85,89,188,186,170,171,187,170,169,153,135,68,51,104,155,203,154,169,153,152,150,35,34,34,38,85,137,151,102,102,102,137,170,170,170,153,153,138,204,203,187,169,153,170,150,69,85,205,221,101,92,98,34,59,237,238,237,98,34,37,67,187,205,166,154,104,101,85,85,85,85,85,85,84,69,85,120,154,102,101,85,86,103,68,68,84,68, 
};