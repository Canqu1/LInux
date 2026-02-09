                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ANSI-C Compiler
                                      3 ; Version 4.0.0 #11528 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module main
                                      6 	.optsdcc -mmcs51 --model-small
                                      7 	
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _main
                                     12 	.globl _Send_Hex_Format
                                     13 	.globl _Send_Binary_String
                                     14 	.globl _Send_JSON_Format
                                     15 	.globl _Send_Individual_Values
                                     16 	.globl _Send_Pattern_Decimal
                                     17 	.globl _Get_Line_Pattern
                                     18 	.globl _Read_Line_Sensors
                                     19 	.globl _UART_Init
                                     20 	.globl _GPIO_Init
                                     21 	.globl _Timer0_Delay1ms
                                     22 	.globl _InitialUART0_Timer1
                                     23 	.globl _printf
                                     24 	.globl _MOSI
                                     25 	.globl _P00
                                     26 	.globl _MISO
                                     27 	.globl _P01
                                     28 	.globl _RXD_1
                                     29 	.globl _P02
                                     30 	.globl _P03
                                     31 	.globl _STADC
                                     32 	.globl _P04
                                     33 	.globl _P05
                                     34 	.globl _TXD
                                     35 	.globl _P06
                                     36 	.globl _RXD
                                     37 	.globl _P07
                                     38 	.globl _IT0
                                     39 	.globl _IE0
                                     40 	.globl _IT1
                                     41 	.globl _IE1
                                     42 	.globl _TR0
                                     43 	.globl _TF0
                                     44 	.globl _TR1
                                     45 	.globl _TF1
                                     46 	.globl _P10
                                     47 	.globl _P11
                                     48 	.globl _P12
                                     49 	.globl _SCL
                                     50 	.globl _P13
                                     51 	.globl _SDA
                                     52 	.globl _P14
                                     53 	.globl _P15
                                     54 	.globl _TXD_1
                                     55 	.globl _P16
                                     56 	.globl _P17
                                     57 	.globl _RI
                                     58 	.globl _TI
                                     59 	.globl _RB8
                                     60 	.globl _TB8
                                     61 	.globl _REN
                                     62 	.globl _SM2
                                     63 	.globl _SM1
                                     64 	.globl _FE
                                     65 	.globl _SM0
                                     66 	.globl _P20
                                     67 	.globl _EX0
                                     68 	.globl _ET0
                                     69 	.globl _EX1
                                     70 	.globl _ET1
                                     71 	.globl _ES
                                     72 	.globl _EBOD
                                     73 	.globl _EADC
                                     74 	.globl _EA
                                     75 	.globl _P30
                                     76 	.globl _PX0
                                     77 	.globl _PT0
                                     78 	.globl _PX1
                                     79 	.globl _PT1
                                     80 	.globl _PS
                                     81 	.globl _PBOD
                                     82 	.globl _PADC
                                     83 	.globl _I2CPX
                                     84 	.globl _AA
                                     85 	.globl _SI
                                     86 	.globl _STO
                                     87 	.globl _STA
                                     88 	.globl _I2CEN
                                     89 	.globl _CM_RL2
                                     90 	.globl _TR2
                                     91 	.globl _TF2
                                     92 	.globl _P
                                     93 	.globl _OV
                                     94 	.globl _RS0
                                     95 	.globl _RS1
                                     96 	.globl _F0
                                     97 	.globl _AC
                                     98 	.globl _CY
                                     99 	.globl _CLRPWM
                                    100 	.globl _PWMF
                                    101 	.globl _LOAD
                                    102 	.globl _PWMRUN
                                    103 	.globl _ADCHS0
                                    104 	.globl _ADCHS1
                                    105 	.globl _ADCHS2
                                    106 	.globl _ADCHS3
                                    107 	.globl _ETGSEL0
                                    108 	.globl _ETGSEL1
                                    109 	.globl _ADCS
                                    110 	.globl _ADCF
                                    111 	.globl _RI_1
                                    112 	.globl _TI_1
                                    113 	.globl _RB8_1
                                    114 	.globl _TB8_1
                                    115 	.globl _REN_1
                                    116 	.globl _SM2_1
                                    117 	.globl _SM1_1
                                    118 	.globl _FE_1
                                    119 	.globl _SM0_1
                                    120 	.globl _EIPH1
                                    121 	.globl _EIP1
                                    122 	.globl _PMD
                                    123 	.globl _PMEN
                                    124 	.globl _PDTCNT
                                    125 	.globl _PDTEN
                                    126 	.globl _SCON_1
                                    127 	.globl _EIPH
                                    128 	.globl _AINDIDS
                                    129 	.globl _SPDR
                                    130 	.globl _SPSR
                                    131 	.globl _SPCR2
                                    132 	.globl _SPCR
                                    133 	.globl _CAPCON4
                                    134 	.globl _CAPCON3
                                    135 	.globl _B
                                    136 	.globl _EIP
                                    137 	.globl _C2H
                                    138 	.globl _C2L
                                    139 	.globl _PIF
                                    140 	.globl _PIPEN
                                    141 	.globl _PINEN
                                    142 	.globl _PICON
                                    143 	.globl _ADCCON0
                                    144 	.globl _C1H
                                    145 	.globl _C1L
                                    146 	.globl _C0H
                                    147 	.globl _C0L
                                    148 	.globl _ADCDLY
                                    149 	.globl _ADCCON2
                                    150 	.globl _ADCCON1
                                    151 	.globl _ACC
                                    152 	.globl _PWMCON1
                                    153 	.globl _PIOCON0
                                    154 	.globl _PWM3L
                                    155 	.globl _PWM2L
                                    156 	.globl _PWM1L
                                    157 	.globl _PWM0L
                                    158 	.globl _PWMPL
                                    159 	.globl _PWMCON0
                                    160 	.globl _FBD
                                    161 	.globl _PNP
                                    162 	.globl _PWM3H
                                    163 	.globl _PWM2H
                                    164 	.globl _PWM1H
                                    165 	.globl _PWM0H
                                    166 	.globl _PWMPH
                                    167 	.globl _PSW
                                    168 	.globl _ADCMPH
                                    169 	.globl _ADCMPL
                                    170 	.globl _PWM5L
                                    171 	.globl _TH2
                                    172 	.globl _PWM4L
                                    173 	.globl _TL2
                                    174 	.globl _RCMP2H
                                    175 	.globl _RCMP2L
                                    176 	.globl _T2MOD
                                    177 	.globl _T2CON
                                    178 	.globl _TA
                                    179 	.globl _PIOCON1
                                    180 	.globl _RH3
                                    181 	.globl _PWM5H
                                    182 	.globl _RL3
                                    183 	.globl _PWM4H
                                    184 	.globl _T3CON
                                    185 	.globl _ADCRH
                                    186 	.globl _ADCRL
                                    187 	.globl _I2ADDR
                                    188 	.globl _I2CON
                                    189 	.globl _I2TOC
                                    190 	.globl _I2CLK
                                    191 	.globl _I2STAT
                                    192 	.globl _I2DAT
                                    193 	.globl _SADDR_1
                                    194 	.globl _SADEN_1
                                    195 	.globl _SADEN
                                    196 	.globl _IP
                                    197 	.globl _PWMINTC
                                    198 	.globl _IPH
                                    199 	.globl _P2S
                                    200 	.globl _P1SR
                                    201 	.globl _P1M2
                                    202 	.globl _P1S
                                    203 	.globl _P1M1
                                    204 	.globl _P0SR
                                    205 	.globl _P0M2
                                    206 	.globl _P0S
                                    207 	.globl _P0M1
                                    208 	.globl _P3
                                    209 	.globl _IAPCN
                                    210 	.globl _IAPFD
                                    211 	.globl _P3SR
                                    212 	.globl _P3M2
                                    213 	.globl _P3S
                                    214 	.globl _P3M1
                                    215 	.globl _BODCON1
                                    216 	.globl _WDCON
                                    217 	.globl _SADDR
                                    218 	.globl _IE
                                    219 	.globl _IAPAH
                                    220 	.globl _IAPAL
                                    221 	.globl _IAPUEN
                                    222 	.globl _IAPTRG
                                    223 	.globl _BODCON0
                                    224 	.globl _AUXR1
                                    225 	.globl _P2
                                    226 	.globl _CHPCON
                                    227 	.globl _EIE1
                                    228 	.globl _EIE
                                    229 	.globl _SBUF_1
                                    230 	.globl _SBUF
                                    231 	.globl _SCON
                                    232 	.globl _CKEN
                                    233 	.globl _CKSWT
                                    234 	.globl _CKDIV
                                    235 	.globl _CAPCON2
                                    236 	.globl _CAPCON1
                                    237 	.globl _CAPCON0
                                    238 	.globl _SFRS
                                    239 	.globl _P1
                                    240 	.globl _WKCON
                                    241 	.globl _CKCON
                                    242 	.globl _TH1
                                    243 	.globl _TH0
                                    244 	.globl _TL1
                                    245 	.globl _TL0
                                    246 	.globl _TMOD
                                    247 	.globl _TCON
                                    248 	.globl _PCON
                                    249 	.globl _RWK
                                    250 	.globl _RCTRIM1
                                    251 	.globl _RCTRIM0
                                    252 	.globl _DPH
                                    253 	.globl _DPL
                                    254 	.globl _SP
                                    255 	.globl _P0
                                    256 	.globl _line_pattern
                                    257 	.globl _line_sensors
                                    258 	.globl _putchar
                                    259 ;--------------------------------------------------------
                                    260 ; special function registers
                                    261 ;--------------------------------------------------------
                                    262 	.area RSEG    (ABS,DATA)
      000000                        263 	.org 0x0000
                           000080   264 _P0	=	0x0080
                           000081   265 _SP	=	0x0081
                           000082   266 _DPL	=	0x0082
                           000083   267 _DPH	=	0x0083
                           000084   268 _RCTRIM0	=	0x0084
                           000085   269 _RCTRIM1	=	0x0085
                           000086   270 _RWK	=	0x0086
                           000087   271 _PCON	=	0x0087
                           000088   272 _TCON	=	0x0088
                           000089   273 _TMOD	=	0x0089
                           00008A   274 _TL0	=	0x008a
                           00008B   275 _TL1	=	0x008b
                           00008C   276 _TH0	=	0x008c
                           00008D   277 _TH1	=	0x008d
                           00008E   278 _CKCON	=	0x008e
                           00008F   279 _WKCON	=	0x008f
                           000090   280 _P1	=	0x0090
                           000091   281 _SFRS	=	0x0091
                           000092   282 _CAPCON0	=	0x0092
                           000093   283 _CAPCON1	=	0x0093
                           000094   284 _CAPCON2	=	0x0094
                           000095   285 _CKDIV	=	0x0095
                           000096   286 _CKSWT	=	0x0096
                           000097   287 _CKEN	=	0x0097
                           000098   288 _SCON	=	0x0098
                           000099   289 _SBUF	=	0x0099
                           00009A   290 _SBUF_1	=	0x009a
                           00009B   291 _EIE	=	0x009b
                           00009C   292 _EIE1	=	0x009c
                           00009F   293 _CHPCON	=	0x009f
                           0000A0   294 _P2	=	0x00a0
                           0000A2   295 _AUXR1	=	0x00a2
                           0000A3   296 _BODCON0	=	0x00a3
                           0000A4   297 _IAPTRG	=	0x00a4
                           0000A5   298 _IAPUEN	=	0x00a5
                           0000A6   299 _IAPAL	=	0x00a6
                           0000A7   300 _IAPAH	=	0x00a7
                           0000A8   301 _IE	=	0x00a8
                           0000A9   302 _SADDR	=	0x00a9
                           0000AA   303 _WDCON	=	0x00aa
                           0000AB   304 _BODCON1	=	0x00ab
                           0000AC   305 _P3M1	=	0x00ac
                           0000AC   306 _P3S	=	0x00ac
                           0000AD   307 _P3M2	=	0x00ad
                           0000AD   308 _P3SR	=	0x00ad
                           0000AE   309 _IAPFD	=	0x00ae
                           0000AF   310 _IAPCN	=	0x00af
                           0000B0   311 _P3	=	0x00b0
                           0000B1   312 _P0M1	=	0x00b1
                           0000B1   313 _P0S	=	0x00b1
                           0000B2   314 _P0M2	=	0x00b2
                           0000B2   315 _P0SR	=	0x00b2
                           0000B3   316 _P1M1	=	0x00b3
                           0000B3   317 _P1S	=	0x00b3
                           0000B4   318 _P1M2	=	0x00b4
                           0000B4   319 _P1SR	=	0x00b4
                           0000B5   320 _P2S	=	0x00b5
                           0000B7   321 _IPH	=	0x00b7
                           0000B7   322 _PWMINTC	=	0x00b7
                           0000B8   323 _IP	=	0x00b8
                           0000B9   324 _SADEN	=	0x00b9
                           0000BA   325 _SADEN_1	=	0x00ba
                           0000BB   326 _SADDR_1	=	0x00bb
                           0000BC   327 _I2DAT	=	0x00bc
                           0000BD   328 _I2STAT	=	0x00bd
                           0000BE   329 _I2CLK	=	0x00be
                           0000BF   330 _I2TOC	=	0x00bf
                           0000C0   331 _I2CON	=	0x00c0
                           0000C1   332 _I2ADDR	=	0x00c1
                           0000C2   333 _ADCRL	=	0x00c2
                           0000C3   334 _ADCRH	=	0x00c3
                           0000C4   335 _T3CON	=	0x00c4
                           0000C4   336 _PWM4H	=	0x00c4
                           0000C5   337 _RL3	=	0x00c5
                           0000C5   338 _PWM5H	=	0x00c5
                           0000C6   339 _RH3	=	0x00c6
                           0000C6   340 _PIOCON1	=	0x00c6
                           0000C7   341 _TA	=	0x00c7
                           0000C8   342 _T2CON	=	0x00c8
                           0000C9   343 _T2MOD	=	0x00c9
                           0000CA   344 _RCMP2L	=	0x00ca
                           0000CB   345 _RCMP2H	=	0x00cb
                           0000CC   346 _TL2	=	0x00cc
                           0000CC   347 _PWM4L	=	0x00cc
                           0000CD   348 _TH2	=	0x00cd
                           0000CD   349 _PWM5L	=	0x00cd
                           0000CE   350 _ADCMPL	=	0x00ce
                           0000CF   351 _ADCMPH	=	0x00cf
                           0000D0   352 _PSW	=	0x00d0
                           0000D1   353 _PWMPH	=	0x00d1
                           0000D2   354 _PWM0H	=	0x00d2
                           0000D3   355 _PWM1H	=	0x00d3
                           0000D4   356 _PWM2H	=	0x00d4
                           0000D5   357 _PWM3H	=	0x00d5
                           0000D6   358 _PNP	=	0x00d6
                           0000D7   359 _FBD	=	0x00d7
                           0000D8   360 _PWMCON0	=	0x00d8
                           0000D9   361 _PWMPL	=	0x00d9
                           0000DA   362 _PWM0L	=	0x00da
                           0000DB   363 _PWM1L	=	0x00db
                           0000DC   364 _PWM2L	=	0x00dc
                           0000DD   365 _PWM3L	=	0x00dd
                           0000DE   366 _PIOCON0	=	0x00de
                           0000DF   367 _PWMCON1	=	0x00df
                           0000E0   368 _ACC	=	0x00e0
                           0000E1   369 _ADCCON1	=	0x00e1
                           0000E2   370 _ADCCON2	=	0x00e2
                           0000E3   371 _ADCDLY	=	0x00e3
                           0000E4   372 _C0L	=	0x00e4
                           0000E5   373 _C0H	=	0x00e5
                           0000E6   374 _C1L	=	0x00e6
                           0000E7   375 _C1H	=	0x00e7
                           0000E8   376 _ADCCON0	=	0x00e8
                           0000E9   377 _PICON	=	0x00e9
                           0000EA   378 _PINEN	=	0x00ea
                           0000EB   379 _PIPEN	=	0x00eb
                           0000EC   380 _PIF	=	0x00ec
                           0000ED   381 _C2L	=	0x00ed
                           0000EE   382 _C2H	=	0x00ee
                           0000EF   383 _EIP	=	0x00ef
                           0000F0   384 _B	=	0x00f0
                           0000F1   385 _CAPCON3	=	0x00f1
                           0000F2   386 _CAPCON4	=	0x00f2
                           0000F3   387 _SPCR	=	0x00f3
                           0000F3   388 _SPCR2	=	0x00f3
                           0000F4   389 _SPSR	=	0x00f4
                           0000F5   390 _SPDR	=	0x00f5
                           0000F6   391 _AINDIDS	=	0x00f6
                           0000F7   392 _EIPH	=	0x00f7
                           0000F8   393 _SCON_1	=	0x00f8
                           0000F9   394 _PDTEN	=	0x00f9
                           0000FA   395 _PDTCNT	=	0x00fa
                           0000FB   396 _PMEN	=	0x00fb
                           0000FC   397 _PMD	=	0x00fc
                           0000FE   398 _EIP1	=	0x00fe
                           0000FF   399 _EIPH1	=	0x00ff
                                    400 ;--------------------------------------------------------
                                    401 ; special function bits
                                    402 ;--------------------------------------------------------
                                    403 	.area RSEG    (ABS,DATA)
      000000                        404 	.org 0x0000
                           0000FF   405 _SM0_1	=	0x00ff
                           0000FF   406 _FE_1	=	0x00ff
                           0000FE   407 _SM1_1	=	0x00fe
                           0000FD   408 _SM2_1	=	0x00fd
                           0000FC   409 _REN_1	=	0x00fc
                           0000FB   410 _TB8_1	=	0x00fb
                           0000FA   411 _RB8_1	=	0x00fa
                           0000F9   412 _TI_1	=	0x00f9
                           0000F8   413 _RI_1	=	0x00f8
                           0000EF   414 _ADCF	=	0x00ef
                           0000EE   415 _ADCS	=	0x00ee
                           0000ED   416 _ETGSEL1	=	0x00ed
                           0000EC   417 _ETGSEL0	=	0x00ec
                           0000EB   418 _ADCHS3	=	0x00eb
                           0000EA   419 _ADCHS2	=	0x00ea
                           0000E9   420 _ADCHS1	=	0x00e9
                           0000E8   421 _ADCHS0	=	0x00e8
                           0000DF   422 _PWMRUN	=	0x00df
                           0000DE   423 _LOAD	=	0x00de
                           0000DD   424 _PWMF	=	0x00dd
                           0000DC   425 _CLRPWM	=	0x00dc
                           0000D7   426 _CY	=	0x00d7
                           0000D6   427 _AC	=	0x00d6
                           0000D5   428 _F0	=	0x00d5
                           0000D4   429 _RS1	=	0x00d4
                           0000D3   430 _RS0	=	0x00d3
                           0000D2   431 _OV	=	0x00d2
                           0000D0   432 _P	=	0x00d0
                           0000CF   433 _TF2	=	0x00cf
                           0000CA   434 _TR2	=	0x00ca
                           0000C8   435 _CM_RL2	=	0x00c8
                           0000C6   436 _I2CEN	=	0x00c6
                           0000C5   437 _STA	=	0x00c5
                           0000C4   438 _STO	=	0x00c4
                           0000C3   439 _SI	=	0x00c3
                           0000C2   440 _AA	=	0x00c2
                           0000C0   441 _I2CPX	=	0x00c0
                           0000BE   442 _PADC	=	0x00be
                           0000BD   443 _PBOD	=	0x00bd
                           0000BC   444 _PS	=	0x00bc
                           0000BB   445 _PT1	=	0x00bb
                           0000BA   446 _PX1	=	0x00ba
                           0000B9   447 _PT0	=	0x00b9
                           0000B8   448 _PX0	=	0x00b8
                           0000B0   449 _P30	=	0x00b0
                           0000AF   450 _EA	=	0x00af
                           0000AE   451 _EADC	=	0x00ae
                           0000AD   452 _EBOD	=	0x00ad
                           0000AC   453 _ES	=	0x00ac
                           0000AB   454 _ET1	=	0x00ab
                           0000AA   455 _EX1	=	0x00aa
                           0000A9   456 _ET0	=	0x00a9
                           0000A8   457 _EX0	=	0x00a8
                           0000A0   458 _P20	=	0x00a0
                           00009F   459 _SM0	=	0x009f
                           00009F   460 _FE	=	0x009f
                           00009E   461 _SM1	=	0x009e
                           00009D   462 _SM2	=	0x009d
                           00009C   463 _REN	=	0x009c
                           00009B   464 _TB8	=	0x009b
                           00009A   465 _RB8	=	0x009a
                           000099   466 _TI	=	0x0099
                           000098   467 _RI	=	0x0098
                           000097   468 _P17	=	0x0097
                           000096   469 _P16	=	0x0096
                           000096   470 _TXD_1	=	0x0096
                           000095   471 _P15	=	0x0095
                           000094   472 _P14	=	0x0094
                           000094   473 _SDA	=	0x0094
                           000093   474 _P13	=	0x0093
                           000093   475 _SCL	=	0x0093
                           000092   476 _P12	=	0x0092
                           000091   477 _P11	=	0x0091
                           000090   478 _P10	=	0x0090
                           00008F   479 _TF1	=	0x008f
                           00008E   480 _TR1	=	0x008e
                           00008D   481 _TF0	=	0x008d
                           00008C   482 _TR0	=	0x008c
                           00008B   483 _IE1	=	0x008b
                           00008A   484 _IT1	=	0x008a
                           000089   485 _IE0	=	0x0089
                           000088   486 _IT0	=	0x0088
                           000087   487 _P07	=	0x0087
                           000087   488 _RXD	=	0x0087
                           000086   489 _P06	=	0x0086
                           000086   490 _TXD	=	0x0086
                           000085   491 _P05	=	0x0085
                           000084   492 _P04	=	0x0084
                           000084   493 _STADC	=	0x0084
                           000083   494 _P03	=	0x0083
                           000082   495 _P02	=	0x0082
                           000082   496 _RXD_1	=	0x0082
                           000081   497 _P01	=	0x0081
                           000081   498 _MISO	=	0x0081
                           000080   499 _P00	=	0x0080
                           000080   500 _MOSI	=	0x0080
                                    501 ;--------------------------------------------------------
                                    502 ; overlayable register banks
                                    503 ;--------------------------------------------------------
                                    504 	.area REG_BANK_0	(REL,OVR,DATA)
      000000                        505 	.ds 8
                                    506 ;--------------------------------------------------------
                                    507 ; internal ram data
                                    508 ;--------------------------------------------------------
                                    509 	.area DSEG    (DATA)
      000008                        510 _line_sensors::
      000008                        511 	.ds 11
      000013                        512 _line_pattern::
      000013                        513 	.ds 2
                                    514 ;--------------------------------------------------------
                                    515 ; overlayable items in internal ram 
                                    516 ;--------------------------------------------------------
                                    517 	.area	OSEG    (OVR,DATA)
                                    518 	.area	OSEG    (OVR,DATA)
                                    519 ;--------------------------------------------------------
                                    520 ; Stack segment in internal ram 
                                    521 ;--------------------------------------------------------
                                    522 	.area	SSEG
      000054                        523 __start__stack:
      000054                        524 	.ds	1
                                    525 
                                    526 ;--------------------------------------------------------
                                    527 ; indirectly addressable internal ram data
                                    528 ;--------------------------------------------------------
                                    529 	.area ISEG    (DATA)
                                    530 ;--------------------------------------------------------
                                    531 ; absolute internal ram data
                                    532 ;--------------------------------------------------------
                                    533 	.area IABS    (ABS,DATA)
                                    534 	.area IABS    (ABS,DATA)
                                    535 ;--------------------------------------------------------
                                    536 ; bit data
                                    537 ;--------------------------------------------------------
                                    538 	.area BSEG    (BIT)
                                    539 ;--------------------------------------------------------
                                    540 ; paged external ram data
                                    541 ;--------------------------------------------------------
                                    542 	.area PSEG    (PAG,XDATA)
                                    543 ;--------------------------------------------------------
                                    544 ; external ram data
                                    545 ;--------------------------------------------------------
                                    546 	.area XSEG    (XDATA)
                                    547 ;--------------------------------------------------------
                                    548 ; absolute external ram data
                                    549 ;--------------------------------------------------------
                                    550 	.area XABS    (ABS,XDATA)
                                    551 ;--------------------------------------------------------
                                    552 ; external initialized ram data
                                    553 ;--------------------------------------------------------
                                    554 	.area HOME    (CODE)
                                    555 	.area GSINIT0 (CODE)
                                    556 	.area GSINIT1 (CODE)
                                    557 	.area GSINIT2 (CODE)
                                    558 	.area GSINIT3 (CODE)
                                    559 	.area GSINIT4 (CODE)
                                    560 	.area GSINIT5 (CODE)
                                    561 	.area GSINIT  (CODE)
                                    562 	.area GSFINAL (CODE)
                                    563 	.area CSEG    (CODE)
                                    564 ;--------------------------------------------------------
                                    565 ; interrupt vector 
                                    566 ;--------------------------------------------------------
                                    567 	.area HOME    (CODE)
      000000                        568 __interrupt_vect:
      000000 02 00 06         [24]  569 	ljmp	__sdcc_gsinit_startup
                                    570 ;--------------------------------------------------------
                                    571 ; global & static initialisations
                                    572 ;--------------------------------------------------------
                                    573 	.area HOME    (CODE)
                                    574 	.area GSINIT  (CODE)
                                    575 	.area GSFINAL (CODE)
                                    576 	.area GSINIT  (CODE)
                                    577 	.globl __sdcc_gsinit_startup
                                    578 	.globl __sdcc_program_startup
                                    579 	.globl __start__stack
                                    580 	.globl __mcs51_genRAMCLEAR
                                    581 ;	src/main.c:23: unsigned int line_pattern = 0;
      000019 E4               [12]  582 	clr	a
      00001A F5 13            [12]  583 	mov	_line_pattern,a
      00001C F5 14            [12]  584 	mov	(_line_pattern + 1),a
                                    585 	.area GSFINAL (CODE)
      00001E 02 00 03         [24]  586 	ljmp	__sdcc_program_startup
                                    587 ;--------------------------------------------------------
                                    588 ; Home
                                    589 ;--------------------------------------------------------
                                    590 	.area HOME    (CODE)
                                    591 	.area HOME    (CODE)
      000003                        592 __sdcc_program_startup:
      000003 02 03 9B         [24]  593 	ljmp	_main
                                    594 ;	return from main will return to caller
                                    595 ;--------------------------------------------------------
                                    596 ; code
                                    597 ;--------------------------------------------------------
                                    598 	.area CSEG    (CODE)
                                    599 ;------------------------------------------------------------
                                    600 ;Allocation info for local variables in function 'putchar'
                                    601 ;------------------------------------------------------------
                                    602 ;c                         Allocated to registers r6 r7 
                                    603 ;------------------------------------------------------------
                                    604 ;	src/main.c:14: int putchar(int c) {
                                    605 ;	-----------------------------------------
                                    606 ;	 function putchar
                                    607 ;	-----------------------------------------
      000021                        608 _putchar:
                           000007   609 	ar7 = 0x07
                           000006   610 	ar6 = 0x06
                           000005   611 	ar5 = 0x05
                           000004   612 	ar4 = 0x04
                           000003   613 	ar3 = 0x03
                           000002   614 	ar2 = 0x02
                           000001   615 	ar1 = 0x01
                           000000   616 	ar0 = 0x00
      000021 AE 82            [24]  617 	mov	r6,dpl
      000023 AF 83            [24]  618 	mov	r7,dph
                                    619 ;	src/main.c:15: while (!TI);
      000025                        620 00101$:
                                    621 ;	src/main.c:16: TI = 0;
                                    622 ;	assignBit
      000025 10 99 02         [24]  623 	jbc	_TI,00114$
      000028 80 FB            [24]  624 	sjmp	00101$
      00002A                        625 00114$:
                                    626 ;	src/main.c:17: SBUF = c;
      00002A 8E 99            [24]  627 	mov	_SBUF,r6
                                    628 ;	src/main.c:18: return c;
      00002C 8E 82            [24]  629 	mov	dpl,r6
      00002E 8F 83            [24]  630 	mov	dph,r7
                                    631 ;	src/main.c:19: }
      000030 22               [24]  632 	ret
                                    633 ;------------------------------------------------------------
                                    634 ;Allocation info for local variables in function 'GPIO_Init'
                                    635 ;------------------------------------------------------------
                                    636 ;	src/main.c:28: void GPIO_Init(void)
                                    637 ;	-----------------------------------------
                                    638 ;	 function GPIO_Init
                                    639 ;	-----------------------------------------
      000031                        640 _GPIO_Init:
                                    641 ;	src/main.c:31: P15_Input_Mode;
      000031 43 B3 20         [24]  642 	orl	_P1M1,#0x20
      000034 53 B4 DF         [24]  643 	anl	_P1M2,#0xdf
                                    644 ;	src/main.c:32: set_P1S_5;
                                    645 ;	assignBit
      000037 A2 AF            [12]  646 	mov	c,_EA
      000039 92 00            [24]  647 	mov	_BIT_TMP,c
      00003B 75 C7 AA         [24]  648 	mov	_TA,#0xaa
      00003E 75 C7 55         [24]  649 	mov	_TA,#0x55
      000041 75 91 01         [24]  650 	mov	_SFRS,#0x01
      000044 43 B3 20         [24]  651 	orl	_P1S,#0x20
      000047 75 C7 AA         [24]  652 	mov	_TA,#0xaa
      00004A 75 C7 55         [24]  653 	mov	_TA,#0x55
      00004D 75 91 00         [24]  654 	mov	_SFRS,#0x00
                                    655 ;	assignBit
      000050 A2 00            [12]  656 	mov	c,_BIT_TMP
      000052 92 AF            [24]  657 	mov	_EA,c
                                    658 ;	src/main.c:35: P05_Input_Mode;
      000054 43 B1 20         [24]  659 	orl	_P0M1,#0x20
      000057 53 B2 DF         [24]  660 	anl	_P0M2,#0xdf
                                    661 ;	src/main.c:36: set_P0S_5;
                                    662 ;	assignBit
      00005A A2 AF            [12]  663 	mov	c,_EA
      00005C 92 00            [24]  664 	mov	_BIT_TMP,c
      00005E 75 C7 AA         [24]  665 	mov	_TA,#0xaa
      000061 75 C7 55         [24]  666 	mov	_TA,#0x55
      000064 75 91 01         [24]  667 	mov	_SFRS,#0x01
      000067 43 B1 20         [24]  668 	orl	_P0S,#0x20
      00006A 75 C7 AA         [24]  669 	mov	_TA,#0xaa
      00006D 75 C7 55         [24]  670 	mov	_TA,#0x55
      000070 75 91 00         [24]  671 	mov	_SFRS,#0x00
                                    672 ;	assignBit
      000073 A2 00            [12]  673 	mov	c,_BIT_TMP
      000075 92 AF            [24]  674 	mov	_EA,c
                                    675 ;	src/main.c:39: P04_Input_Mode;
      000077 43 B1 10         [24]  676 	orl	_P0M1,#0x10
      00007A 53 B2 EF         [24]  677 	anl	_P0M2,#0xef
                                    678 ;	src/main.c:40: set_P0S_4;
                                    679 ;	assignBit
      00007D A2 AF            [12]  680 	mov	c,_EA
      00007F 92 00            [24]  681 	mov	_BIT_TMP,c
      000081 75 C7 AA         [24]  682 	mov	_TA,#0xaa
      000084 75 C7 55         [24]  683 	mov	_TA,#0x55
      000087 75 91 01         [24]  684 	mov	_SFRS,#0x01
      00008A 43 B1 10         [24]  685 	orl	_P0S,#0x10
      00008D 75 C7 AA         [24]  686 	mov	_TA,#0xaa
      000090 75 C7 55         [24]  687 	mov	_TA,#0x55
      000093 75 91 00         [24]  688 	mov	_SFRS,#0x00
                                    689 ;	assignBit
      000096 A2 00            [12]  690 	mov	c,_BIT_TMP
      000098 92 AF            [24]  691 	mov	_EA,c
                                    692 ;	src/main.c:43: P03_Input_Mode;
      00009A 43 B1 08         [24]  693 	orl	_P0M1,#0x08
      00009D 53 B2 F7         [24]  694 	anl	_P0M2,#0xf7
                                    695 ;	src/main.c:44: set_P0S_3;
                                    696 ;	assignBit
      0000A0 A2 AF            [12]  697 	mov	c,_EA
      0000A2 92 00            [24]  698 	mov	_BIT_TMP,c
      0000A4 75 C7 AA         [24]  699 	mov	_TA,#0xaa
      0000A7 75 C7 55         [24]  700 	mov	_TA,#0x55
      0000AA 75 91 01         [24]  701 	mov	_SFRS,#0x01
      0000AD 43 B1 08         [24]  702 	orl	_P0S,#0x08
      0000B0 75 C7 AA         [24]  703 	mov	_TA,#0xaa
      0000B3 75 C7 55         [24]  704 	mov	_TA,#0x55
      0000B6 75 91 00         [24]  705 	mov	_SFRS,#0x00
                                    706 ;	assignBit
      0000B9 A2 00            [12]  707 	mov	c,_BIT_TMP
      0000BB 92 AF            [24]  708 	mov	_EA,c
                                    709 ;	src/main.c:47: P01_Input_Mode;
      0000BD 43 B1 02         [24]  710 	orl	_P0M1,#0x02
      0000C0 53 B2 FD         [24]  711 	anl	_P0M2,#0xfd
                                    712 ;	src/main.c:48: set_P0S_1;
                                    713 ;	assignBit
      0000C3 A2 AF            [12]  714 	mov	c,_EA
      0000C5 92 00            [24]  715 	mov	_BIT_TMP,c
      0000C7 75 C7 AA         [24]  716 	mov	_TA,#0xaa
      0000CA 75 C7 55         [24]  717 	mov	_TA,#0x55
      0000CD 75 91 01         [24]  718 	mov	_SFRS,#0x01
      0000D0 43 B1 02         [24]  719 	orl	_P0S,#0x02
      0000D3 75 C7 AA         [24]  720 	mov	_TA,#0xaa
      0000D6 75 C7 55         [24]  721 	mov	_TA,#0x55
      0000D9 75 91 00         [24]  722 	mov	_SFRS,#0x00
                                    723 ;	assignBit
      0000DC A2 00            [12]  724 	mov	c,_BIT_TMP
      0000DE 92 AF            [24]  725 	mov	_EA,c
                                    726 ;	src/main.c:51: P00_Input_Mode;
      0000E0 43 B1 01         [24]  727 	orl	_P0M1,#0x01
      0000E3 53 B2 FE         [24]  728 	anl	_P0M2,#0xfe
                                    729 ;	src/main.c:52: set_P0S_0;
                                    730 ;	assignBit
      0000E6 A2 AF            [12]  731 	mov	c,_EA
      0000E8 92 00            [24]  732 	mov	_BIT_TMP,c
      0000EA 75 C7 AA         [24]  733 	mov	_TA,#0xaa
      0000ED 75 C7 55         [24]  734 	mov	_TA,#0x55
      0000F0 75 91 01         [24]  735 	mov	_SFRS,#0x01
      0000F3 43 B1 01         [24]  736 	orl	_P0S,#0x01
      0000F6 75 C7 AA         [24]  737 	mov	_TA,#0xaa
      0000F9 75 C7 55         [24]  738 	mov	_TA,#0x55
      0000FC 75 91 00         [24]  739 	mov	_SFRS,#0x00
                                    740 ;	assignBit
      0000FF A2 00            [12]  741 	mov	c,_BIT_TMP
      000101 92 AF            [24]  742 	mov	_EA,c
                                    743 ;	src/main.c:55: P10_Input_Mode;
      000103 43 B3 01         [24]  744 	orl	_P1M1,#0x01
      000106 53 B4 FE         [24]  745 	anl	_P1M2,#0xfe
                                    746 ;	src/main.c:56: set_P1S_0;
                                    747 ;	assignBit
      000109 A2 AF            [12]  748 	mov	c,_EA
      00010B 92 00            [24]  749 	mov	_BIT_TMP,c
      00010D 75 C7 AA         [24]  750 	mov	_TA,#0xaa
      000110 75 C7 55         [24]  751 	mov	_TA,#0x55
      000113 75 91 01         [24]  752 	mov	_SFRS,#0x01
      000116 43 B3 01         [24]  753 	orl	_P1S,#0x01
      000119 75 C7 AA         [24]  754 	mov	_TA,#0xaa
      00011C 75 C7 55         [24]  755 	mov	_TA,#0x55
      00011F 75 91 00         [24]  756 	mov	_SFRS,#0x00
                                    757 ;	assignBit
      000122 A2 00            [12]  758 	mov	c,_BIT_TMP
      000124 92 AF            [24]  759 	mov	_EA,c
                                    760 ;	src/main.c:59: P11_Input_Mode;
      000126 43 B3 02         [24]  761 	orl	_P1M1,#0x02
      000129 53 B4 FD         [24]  762 	anl	_P1M2,#0xfd
                                    763 ;	src/main.c:60: set_P1S_1;
                                    764 ;	assignBit
      00012C A2 AF            [12]  765 	mov	c,_EA
      00012E 92 00            [24]  766 	mov	_BIT_TMP,c
      000130 75 C7 AA         [24]  767 	mov	_TA,#0xaa
      000133 75 C7 55         [24]  768 	mov	_TA,#0x55
      000136 75 91 01         [24]  769 	mov	_SFRS,#0x01
      000139 43 B3 02         [24]  770 	orl	_P1S,#0x02
      00013C 75 C7 AA         [24]  771 	mov	_TA,#0xaa
      00013F 75 C7 55         [24]  772 	mov	_TA,#0x55
      000142 75 91 00         [24]  773 	mov	_SFRS,#0x00
                                    774 ;	assignBit
      000145 A2 00            [12]  775 	mov	c,_BIT_TMP
      000147 92 AF            [24]  776 	mov	_EA,c
                                    777 ;	src/main.c:63: P12_Input_Mode;
      000149 43 B3 04         [24]  778 	orl	_P1M1,#0x04
      00014C 53 B4 FB         [24]  779 	anl	_P1M2,#0xfb
                                    780 ;	src/main.c:64: set_P1S_2;
                                    781 ;	assignBit
      00014F A2 AF            [12]  782 	mov	c,_EA
      000151 92 00            [24]  783 	mov	_BIT_TMP,c
      000153 75 C7 AA         [24]  784 	mov	_TA,#0xaa
      000156 75 C7 55         [24]  785 	mov	_TA,#0x55
      000159 75 91 01         [24]  786 	mov	_SFRS,#0x01
      00015C 43 B3 04         [24]  787 	orl	_P1S,#0x04
      00015F 75 C7 AA         [24]  788 	mov	_TA,#0xaa
      000162 75 C7 55         [24]  789 	mov	_TA,#0x55
      000165 75 91 00         [24]  790 	mov	_SFRS,#0x00
                                    791 ;	assignBit
      000168 A2 00            [12]  792 	mov	c,_BIT_TMP
      00016A 92 AF            [24]  793 	mov	_EA,c
                                    794 ;	src/main.c:67: P13_Input_Mode;
      00016C 43 B3 08         [24]  795 	orl	_P1M1,#0x08
      00016F 53 B4 F7         [24]  796 	anl	_P1M2,#0xf7
                                    797 ;	src/main.c:68: set_P1S_3;
                                    798 ;	assignBit
      000172 A2 AF            [12]  799 	mov	c,_EA
      000174 92 00            [24]  800 	mov	_BIT_TMP,c
      000176 75 C7 AA         [24]  801 	mov	_TA,#0xaa
      000179 75 C7 55         [24]  802 	mov	_TA,#0x55
      00017C 75 91 01         [24]  803 	mov	_SFRS,#0x01
      00017F 43 B3 08         [24]  804 	orl	_P1S,#0x08
      000182 75 C7 AA         [24]  805 	mov	_TA,#0xaa
      000185 75 C7 55         [24]  806 	mov	_TA,#0x55
      000188 75 91 00         [24]  807 	mov	_SFRS,#0x00
                                    808 ;	assignBit
      00018B A2 00            [12]  809 	mov	c,_BIT_TMP
      00018D 92 AF            [24]  810 	mov	_EA,c
                                    811 ;	src/main.c:71: P14_Input_Mode;
      00018F 43 B3 10         [24]  812 	orl	_P1M1,#0x10
      000192 53 B4 EF         [24]  813 	anl	_P1M2,#0xef
                                    814 ;	src/main.c:72: set_P1S_4;
                                    815 ;	assignBit
      000195 A2 AF            [12]  816 	mov	c,_EA
      000197 92 00            [24]  817 	mov	_BIT_TMP,c
      000199 75 C7 AA         [24]  818 	mov	_TA,#0xaa
      00019C 75 C7 55         [24]  819 	mov	_TA,#0x55
      00019F 75 91 01         [24]  820 	mov	_SFRS,#0x01
      0001A2 43 B3 10         [24]  821 	orl	_P1S,#0x10
      0001A5 75 C7 AA         [24]  822 	mov	_TA,#0xaa
      0001A8 75 C7 55         [24]  823 	mov	_TA,#0x55
      0001AB 75 91 00         [24]  824 	mov	_SFRS,#0x00
                                    825 ;	assignBit
      0001AE A2 00            [12]  826 	mov	c,_BIT_TMP
      0001B0 92 AF            [24]  827 	mov	_EA,c
                                    828 ;	src/main.c:73: }
      0001B2 22               [24]  829 	ret
                                    830 ;------------------------------------------------------------
                                    831 ;Allocation info for local variables in function 'UART_Init'
                                    832 ;------------------------------------------------------------
                                    833 ;	src/main.c:78: void UART_Init(void)
                                    834 ;	-----------------------------------------
                                    835 ;	 function UART_Init
                                    836 ;	-----------------------------------------
      0001B3                        837 _UART_Init:
                                    838 ;	src/main.c:80: InitialUART0_Timer1(115200);  // 115200 baud rate
      0001B3 90 C2 00         [24]  839 	mov	dptr,#0xc200
      0001B6 75 F0 01         [24]  840 	mov	b,#0x01
      0001B9 E4               [12]  841 	clr	a
                                    842 ;	src/main.c:81: }
      0001BA 02 03 EB         [24]  843 	ljmp	_InitialUART0_Timer1
                                    844 ;------------------------------------------------------------
                                    845 ;Allocation info for local variables in function 'Read_Line_Sensors'
                                    846 ;------------------------------------------------------------
                                    847 ;	src/main.c:86: void Read_Line_Sensors(void)
                                    848 ;	-----------------------------------------
                                    849 ;	 function Read_Line_Sensors
                                    850 ;	-----------------------------------------
      0001BD                        851 _Read_Line_Sensors:
                                    852 ;	src/main.c:88: line_sensors[0]  = P15;  // LINE1
      0001BD A2 95            [12]  853 	mov	c,_P15
      0001BF E4               [12]  854 	clr	a
      0001C0 33               [12]  855 	rlc	a
      0001C1 FF               [12]  856 	mov	r7,a
      0001C2 8F 08            [24]  857 	mov	_line_sensors,r7
                                    858 ;	src/main.c:89: line_sensors[1]  = P05;  // LINE2
      0001C4 A2 85            [12]  859 	mov	c,_P05
      0001C6 E4               [12]  860 	clr	a
      0001C7 33               [12]  861 	rlc	a
      0001C8 FF               [12]  862 	mov	r7,a
      0001C9 8F 09            [24]  863 	mov	(_line_sensors + 0x0001),r7
                                    864 ;	src/main.c:90: line_sensors[2]  = P04;  // LINE3
      0001CB A2 84            [12]  865 	mov	c,_P04
      0001CD E4               [12]  866 	clr	a
      0001CE 33               [12]  867 	rlc	a
      0001CF FF               [12]  868 	mov	r7,a
      0001D0 8F 0A            [24]  869 	mov	(_line_sensors + 0x0002),r7
                                    870 ;	src/main.c:91: line_sensors[3]  = P03;  // LINE4
      0001D2 A2 83            [12]  871 	mov	c,_P03
      0001D4 E4               [12]  872 	clr	a
      0001D5 33               [12]  873 	rlc	a
      0001D6 FF               [12]  874 	mov	r7,a
      0001D7 8F 0B            [24]  875 	mov	(_line_sensors + 0x0003),r7
                                    876 ;	src/main.c:92: line_sensors[4]  = P01;  // LINE5
      0001D9 A2 81            [12]  877 	mov	c,_P01
      0001DB E4               [12]  878 	clr	a
      0001DC 33               [12]  879 	rlc	a
      0001DD FF               [12]  880 	mov	r7,a
      0001DE 8F 0C            [24]  881 	mov	(_line_sensors + 0x0004),r7
                                    882 ;	src/main.c:93: line_sensors[5]  = P00;  // LINE6
      0001E0 A2 80            [12]  883 	mov	c,_P00
      0001E2 E4               [12]  884 	clr	a
      0001E3 33               [12]  885 	rlc	a
      0001E4 FF               [12]  886 	mov	r7,a
      0001E5 8F 0D            [24]  887 	mov	(_line_sensors + 0x0005),r7
                                    888 ;	src/main.c:94: line_sensors[6]  = P10;  // LINE7
      0001E7 A2 90            [12]  889 	mov	c,_P10
      0001E9 E4               [12]  890 	clr	a
      0001EA 33               [12]  891 	rlc	a
      0001EB FF               [12]  892 	mov	r7,a
      0001EC 8F 0E            [24]  893 	mov	(_line_sensors + 0x0006),r7
                                    894 ;	src/main.c:95: line_sensors[7]  = P11;  // LINE8
      0001EE A2 91            [12]  895 	mov	c,_P11
      0001F0 E4               [12]  896 	clr	a
      0001F1 33               [12]  897 	rlc	a
      0001F2 FF               [12]  898 	mov	r7,a
      0001F3 8F 0F            [24]  899 	mov	(_line_sensors + 0x0007),r7
                                    900 ;	src/main.c:96: line_sensors[8]  = P12;  // LINE9
      0001F5 A2 92            [12]  901 	mov	c,_P12
      0001F7 E4               [12]  902 	clr	a
      0001F8 33               [12]  903 	rlc	a
      0001F9 FF               [12]  904 	mov	r7,a
      0001FA 8F 10            [24]  905 	mov	(_line_sensors + 0x0008),r7
                                    906 ;	src/main.c:97: line_sensors[9]  = P13;  // LINE10
      0001FC A2 93            [12]  907 	mov	c,_P13
      0001FE E4               [12]  908 	clr	a
      0001FF 33               [12]  909 	rlc	a
      000200 FF               [12]  910 	mov	r7,a
      000201 8F 11            [24]  911 	mov	(_line_sensors + 0x0009),r7
                                    912 ;	src/main.c:98: line_sensors[10] = P14;  // LINE11
      000203 A2 94            [12]  913 	mov	c,_P14
      000205 E4               [12]  914 	clr	a
      000206 33               [12]  915 	rlc	a
      000207 FF               [12]  916 	mov	r7,a
      000208 8F 12            [24]  917 	mov	(_line_sensors + 0x000a),r7
                                    918 ;	src/main.c:99: }
      00020A 22               [24]  919 	ret
                                    920 ;------------------------------------------------------------
                                    921 ;Allocation info for local variables in function 'Get_Line_Pattern'
                                    922 ;------------------------------------------------------------
                                    923 ;pattern                   Allocated to registers r6 r7 
                                    924 ;i                         Allocated to registers r5 
                                    925 ;------------------------------------------------------------
                                    926 ;	src/main.c:104: unsigned int Get_Line_Pattern(void)
                                    927 ;	-----------------------------------------
                                    928 ;	 function Get_Line_Pattern
                                    929 ;	-----------------------------------------
      00020B                        930 _Get_Line_Pattern:
                                    931 ;	src/main.c:106: unsigned int pattern = 0;
      00020B 7E 00            [12]  932 	mov	r6,#0x00
      00020D 7F 00            [12]  933 	mov	r7,#0x00
                                    934 ;	src/main.c:109: for(i = 0; i < 11; i++)
      00020F 7D 00            [12]  935 	mov	r5,#0x00
      000211                        936 00104$:
                                    937 ;	src/main.c:111: if(line_sensors[i] == 1)
      000211 ED               [12]  938 	mov	a,r5
      000212 24 08            [12]  939 	add	a,#_line_sensors
      000214 F9               [12]  940 	mov	r1,a
      000215 87 04            [24]  941 	mov	ar4,@r1
      000217 BC 01 19         [24]  942 	cjne	r4,#0x01,00105$
                                    943 ;	src/main.c:112: pattern |= (1 << i);
      00021A 8D F0            [24]  944 	mov	b,r5
      00021C 05 F0            [12]  945 	inc	b
      00021E 7B 01            [12]  946 	mov	r3,#0x01
      000220 7C 00            [12]  947 	mov	r4,#0x00
      000222 80 06            [24]  948 	sjmp	00124$
      000224                        949 00123$:
      000224 EB               [12]  950 	mov	a,r3
      000225 2B               [12]  951 	add	a,r3
      000226 FB               [12]  952 	mov	r3,a
      000227 EC               [12]  953 	mov	a,r4
      000228 33               [12]  954 	rlc	a
      000229 FC               [12]  955 	mov	r4,a
      00022A                        956 00124$:
      00022A D5 F0 F7         [24]  957 	djnz	b,00123$
      00022D EB               [12]  958 	mov	a,r3
      00022E 42 06            [12]  959 	orl	ar6,a
      000230 EC               [12]  960 	mov	a,r4
      000231 42 07            [12]  961 	orl	ar7,a
      000233                        962 00105$:
                                    963 ;	src/main.c:109: for(i = 0; i < 11; i++)
      000233 0D               [12]  964 	inc	r5
      000234 BD 0B 00         [24]  965 	cjne	r5,#0x0b,00125$
      000237                        966 00125$:
      000237 40 D8            [24]  967 	jc	00104$
                                    968 ;	src/main.c:115: return pattern;
      000239 8E 82            [24]  969 	mov	dpl,r6
      00023B 8F 83            [24]  970 	mov	dph,r7
                                    971 ;	src/main.c:116: }
      00023D 22               [24]  972 	ret
                                    973 ;------------------------------------------------------------
                                    974 ;Allocation info for local variables in function 'Send_Pattern_Decimal'
                                    975 ;------------------------------------------------------------
                                    976 ;	src/main.c:122: void Send_Pattern_Decimal(void)
                                    977 ;	-----------------------------------------
                                    978 ;	 function Send_Pattern_Decimal
                                    979 ;	-----------------------------------------
      00023E                        980 _Send_Pattern_Decimal:
                                    981 ;	src/main.c:124: printf("%u\r\n", line_pattern);
      00023E C0 13            [24]  982 	push	_line_pattern
      000240 C0 14            [24]  983 	push	(_line_pattern + 1)
      000242 74 D8            [12]  984 	mov	a,#___str_0
      000244 C0 E0            [24]  985 	push	acc
      000246 74 0F            [12]  986 	mov	a,#(___str_0 >> 8)
      000248 C0 E0            [24]  987 	push	acc
      00024A 74 80            [12]  988 	mov	a,#0x80
      00024C C0 E0            [24]  989 	push	acc
      00024E 12 06 D9         [24]  990 	lcall	_printf
      000251 E5 81            [12]  991 	mov	a,sp
      000253 24 FB            [12]  992 	add	a,#0xfb
      000255 F5 81            [12]  993 	mov	sp,a
                                    994 ;	src/main.c:125: }
      000257 22               [24]  995 	ret
                                    996 ;------------------------------------------------------------
                                    997 ;Allocation info for local variables in function 'Send_Individual_Values'
                                    998 ;------------------------------------------------------------
                                    999 ;i                         Allocated to registers r7 
                                   1000 ;------------------------------------------------------------
                                   1001 ;	src/main.c:131: void Send_Individual_Values(void)
                                   1002 ;	-----------------------------------------
                                   1003 ;	 function Send_Individual_Values
                                   1004 ;	-----------------------------------------
      000258                       1005 _Send_Individual_Values:
                                   1006 ;	src/main.c:135: for(i = 0; i < 11; i++)
      000258 7F 00            [12] 1007 	mov	r7,#0x00
      00025A                       1008 00104$:
                                   1009 ;	src/main.c:137: printf("%u", line_sensors[i]);
      00025A EF               [12] 1010 	mov	a,r7
      00025B 24 08            [12] 1011 	add	a,#_line_sensors
      00025D F9               [12] 1012 	mov	r1,a
      00025E 87 06            [24] 1013 	mov	ar6,@r1
      000260 7D 00            [12] 1014 	mov	r5,#0x00
      000262 C0 07            [24] 1015 	push	ar7
      000264 C0 06            [24] 1016 	push	ar6
      000266 C0 05            [24] 1017 	push	ar5
      000268 74 DD            [12] 1018 	mov	a,#___str_1
      00026A C0 E0            [24] 1019 	push	acc
      00026C 74 0F            [12] 1020 	mov	a,#(___str_1 >> 8)
      00026E C0 E0            [24] 1021 	push	acc
      000270 74 80            [12] 1022 	mov	a,#0x80
      000272 C0 E0            [24] 1023 	push	acc
      000274 12 06 D9         [24] 1024 	lcall	_printf
      000277 E5 81            [12] 1025 	mov	a,sp
      000279 24 FB            [12] 1026 	add	a,#0xfb
      00027B F5 81            [12] 1027 	mov	sp,a
      00027D D0 07            [24] 1028 	pop	ar7
                                   1029 ;	src/main.c:138: if(i < 10)
      00027F BF 0A 00         [24] 1030 	cjne	r7,#0x0a,00121$
      000282                       1031 00121$:
      000282 50 19            [24] 1032 	jnc	00105$
                                   1033 ;	src/main.c:139: printf(" ");
      000284 C0 07            [24] 1034 	push	ar7
      000286 74 E0            [12] 1035 	mov	a,#___str_2
      000288 C0 E0            [24] 1036 	push	acc
      00028A 74 0F            [12] 1037 	mov	a,#(___str_2 >> 8)
      00028C C0 E0            [24] 1038 	push	acc
      00028E 74 80            [12] 1039 	mov	a,#0x80
      000290 C0 E0            [24] 1040 	push	acc
      000292 12 06 D9         [24] 1041 	lcall	_printf
      000295 15 81            [12] 1042 	dec	sp
      000297 15 81            [12] 1043 	dec	sp
      000299 15 81            [12] 1044 	dec	sp
      00029B D0 07            [24] 1045 	pop	ar7
      00029D                       1046 00105$:
                                   1047 ;	src/main.c:135: for(i = 0; i < 11; i++)
      00029D 0F               [12] 1048 	inc	r7
      00029E BF 0B 00         [24] 1049 	cjne	r7,#0x0b,00123$
      0002A1                       1050 00123$:
      0002A1 40 B7            [24] 1051 	jc	00104$
                                   1052 ;	src/main.c:141: printf("\r\n");
      0002A3 74 E2            [12] 1053 	mov	a,#___str_3
      0002A5 C0 E0            [24] 1054 	push	acc
      0002A7 74 0F            [12] 1055 	mov	a,#(___str_3 >> 8)
      0002A9 C0 E0            [24] 1056 	push	acc
      0002AB 74 80            [12] 1057 	mov	a,#0x80
      0002AD C0 E0            [24] 1058 	push	acc
      0002AF 12 06 D9         [24] 1059 	lcall	_printf
      0002B2 15 81            [12] 1060 	dec	sp
      0002B4 15 81            [12] 1061 	dec	sp
      0002B6 15 81            [12] 1062 	dec	sp
                                   1063 ;	src/main.c:142: }
      0002B8 22               [24] 1064 	ret
                                   1065 ;------------------------------------------------------------
                                   1066 ;Allocation info for local variables in function 'Send_JSON_Format'
                                   1067 ;------------------------------------------------------------
                                   1068 ;i                         Allocated to registers r7 
                                   1069 ;------------------------------------------------------------
                                   1070 ;	src/main.c:148: void Send_JSON_Format(void)
                                   1071 ;	-----------------------------------------
                                   1072 ;	 function Send_JSON_Format
                                   1073 ;	-----------------------------------------
      0002B9                       1074 _Send_JSON_Format:
                                   1075 ;	src/main.c:150: printf("{\"sensors\":[");
      0002B9 74 E5            [12] 1076 	mov	a,#___str_4
      0002BB C0 E0            [24] 1077 	push	acc
      0002BD 74 0F            [12] 1078 	mov	a,#(___str_4 >> 8)
      0002BF C0 E0            [24] 1079 	push	acc
      0002C1 74 80            [12] 1080 	mov	a,#0x80
      0002C3 C0 E0            [24] 1081 	push	acc
      0002C5 12 06 D9         [24] 1082 	lcall	_printf
      0002C8 15 81            [12] 1083 	dec	sp
      0002CA 15 81            [12] 1084 	dec	sp
      0002CC 15 81            [12] 1085 	dec	sp
                                   1086 ;	src/main.c:152: for(unsigned char i = 0; i < 11; i++)
      0002CE 7F 00            [12] 1087 	mov	r7,#0x00
      0002D0                       1088 00105$:
      0002D0 BF 0B 00         [24] 1089 	cjne	r7,#0x0b,00122$
      0002D3                       1090 00122$:
      0002D3 50 46            [24] 1091 	jnc	00103$
                                   1092 ;	src/main.c:154: printf("%u", line_sensors[i]);
      0002D5 EF               [12] 1093 	mov	a,r7
      0002D6 24 08            [12] 1094 	add	a,#_line_sensors
      0002D8 F9               [12] 1095 	mov	r1,a
      0002D9 87 06            [24] 1096 	mov	ar6,@r1
      0002DB 7D 00            [12] 1097 	mov	r5,#0x00
      0002DD C0 07            [24] 1098 	push	ar7
      0002DF C0 06            [24] 1099 	push	ar6
      0002E1 C0 05            [24] 1100 	push	ar5
      0002E3 74 DD            [12] 1101 	mov	a,#___str_1
      0002E5 C0 E0            [24] 1102 	push	acc
      0002E7 74 0F            [12] 1103 	mov	a,#(___str_1 >> 8)
      0002E9 C0 E0            [24] 1104 	push	acc
      0002EB 74 80            [12] 1105 	mov	a,#0x80
      0002ED C0 E0            [24] 1106 	push	acc
      0002EF 12 06 D9         [24] 1107 	lcall	_printf
      0002F2 E5 81            [12] 1108 	mov	a,sp
      0002F4 24 FB            [12] 1109 	add	a,#0xfb
      0002F6 F5 81            [12] 1110 	mov	sp,a
      0002F8 D0 07            [24] 1111 	pop	ar7
                                   1112 ;	src/main.c:155: if(i < 10)
      0002FA BF 0A 00         [24] 1113 	cjne	r7,#0x0a,00124$
      0002FD                       1114 00124$:
      0002FD 50 19            [24] 1115 	jnc	00106$
                                   1116 ;	src/main.c:156: printf(",");
      0002FF C0 07            [24] 1117 	push	ar7
      000301 74 F2            [12] 1118 	mov	a,#___str_5
      000303 C0 E0            [24] 1119 	push	acc
      000305 74 0F            [12] 1120 	mov	a,#(___str_5 >> 8)
      000307 C0 E0            [24] 1121 	push	acc
      000309 74 80            [12] 1122 	mov	a,#0x80
      00030B C0 E0            [24] 1123 	push	acc
      00030D 12 06 D9         [24] 1124 	lcall	_printf
      000310 15 81            [12] 1125 	dec	sp
      000312 15 81            [12] 1126 	dec	sp
      000314 15 81            [12] 1127 	dec	sp
      000316 D0 07            [24] 1128 	pop	ar7
      000318                       1129 00106$:
                                   1130 ;	src/main.c:152: for(unsigned char i = 0; i < 11; i++)
      000318 0F               [12] 1131 	inc	r7
      000319 80 B5            [24] 1132 	sjmp	00105$
      00031B                       1133 00103$:
                                   1134 ;	src/main.c:159: printf("],\"pattern\":%u}\r\n", line_pattern);
      00031B C0 13            [24] 1135 	push	_line_pattern
      00031D C0 14            [24] 1136 	push	(_line_pattern + 1)
      00031F 74 F4            [12] 1137 	mov	a,#___str_6
      000321 C0 E0            [24] 1138 	push	acc
      000323 74 0F            [12] 1139 	mov	a,#(___str_6 >> 8)
      000325 C0 E0            [24] 1140 	push	acc
      000327 74 80            [12] 1141 	mov	a,#0x80
      000329 C0 E0            [24] 1142 	push	acc
      00032B 12 06 D9         [24] 1143 	lcall	_printf
      00032E E5 81            [12] 1144 	mov	a,sp
      000330 24 FB            [12] 1145 	add	a,#0xfb
      000332 F5 81            [12] 1146 	mov	sp,a
                                   1147 ;	src/main.c:160: }
      000334 22               [24] 1148 	ret
                                   1149 ;------------------------------------------------------------
                                   1150 ;Allocation info for local variables in function 'Send_Binary_String'
                                   1151 ;------------------------------------------------------------
                                   1152 ;i                         Allocated to registers r7 
                                   1153 ;------------------------------------------------------------
                                   1154 ;	src/main.c:166: void Send_Binary_String(void)
                                   1155 ;	-----------------------------------------
                                   1156 ;	 function Send_Binary_String
                                   1157 ;	-----------------------------------------
      000335                       1158 _Send_Binary_String:
                                   1159 ;	src/main.c:170: for(i = 0; i < 11; i++)
      000335 7F 00            [12] 1160 	mov	r7,#0x00
      000337                       1161 00102$:
                                   1162 ;	src/main.c:172: printf("%c", line_sensors[i] ? '1' : '0');
      000337 EF               [12] 1163 	mov	a,r7
      000338 24 08            [12] 1164 	add	a,#_line_sensors
      00033A F9               [12] 1165 	mov	r1,a
      00033B E7               [12] 1166 	mov	a,@r1
      00033C 60 06            [24] 1167 	jz	00106$
      00033E 7D 31            [12] 1168 	mov	r5,#0x31
      000340 7E 00            [12] 1169 	mov	r6,#0x00
      000342 80 04            [24] 1170 	sjmp	00107$
      000344                       1171 00106$:
      000344 7D 30            [12] 1172 	mov	r5,#0x30
      000346 7E 00            [12] 1173 	mov	r6,#0x00
      000348                       1174 00107$:
      000348 C0 07            [24] 1175 	push	ar7
      00034A C0 05            [24] 1176 	push	ar5
      00034C C0 06            [24] 1177 	push	ar6
      00034E 74 06            [12] 1178 	mov	a,#___str_7
      000350 C0 E0            [24] 1179 	push	acc
      000352 74 10            [12] 1180 	mov	a,#(___str_7 >> 8)
      000354 C0 E0            [24] 1181 	push	acc
      000356 74 80            [12] 1182 	mov	a,#0x80
      000358 C0 E0            [24] 1183 	push	acc
      00035A 12 06 D9         [24] 1184 	lcall	_printf
      00035D E5 81            [12] 1185 	mov	a,sp
      00035F 24 FB            [12] 1186 	add	a,#0xfb
      000361 F5 81            [12] 1187 	mov	sp,a
      000363 D0 07            [24] 1188 	pop	ar7
                                   1189 ;	src/main.c:170: for(i = 0; i < 11; i++)
      000365 0F               [12] 1190 	inc	r7
      000366 BF 0B 00         [24] 1191 	cjne	r7,#0x0b,00122$
      000369                       1192 00122$:
      000369 40 CC            [24] 1193 	jc	00102$
                                   1194 ;	src/main.c:174: printf("\r\n");
      00036B 74 E2            [12] 1195 	mov	a,#___str_3
      00036D C0 E0            [24] 1196 	push	acc
      00036F 74 0F            [12] 1197 	mov	a,#(___str_3 >> 8)
      000371 C0 E0            [24] 1198 	push	acc
      000373 74 80            [12] 1199 	mov	a,#0x80
      000375 C0 E0            [24] 1200 	push	acc
      000377 12 06 D9         [24] 1201 	lcall	_printf
      00037A 15 81            [12] 1202 	dec	sp
      00037C 15 81            [12] 1203 	dec	sp
      00037E 15 81            [12] 1204 	dec	sp
                                   1205 ;	src/main.c:175: }
      000380 22               [24] 1206 	ret
                                   1207 ;------------------------------------------------------------
                                   1208 ;Allocation info for local variables in function 'Send_Hex_Format'
                                   1209 ;------------------------------------------------------------
                                   1210 ;	src/main.c:181: void Send_Hex_Format(void)
                                   1211 ;	-----------------------------------------
                                   1212 ;	 function Send_Hex_Format
                                   1213 ;	-----------------------------------------
      000381                       1214 _Send_Hex_Format:
                                   1215 ;	src/main.c:183: printf("L:%03X\r\n", line_pattern);
      000381 C0 13            [24] 1216 	push	_line_pattern
      000383 C0 14            [24] 1217 	push	(_line_pattern + 1)
      000385 74 09            [12] 1218 	mov	a,#___str_8
      000387 C0 E0            [24] 1219 	push	acc
      000389 74 10            [12] 1220 	mov	a,#(___str_8 >> 8)
      00038B C0 E0            [24] 1221 	push	acc
      00038D 74 80            [12] 1222 	mov	a,#0x80
      00038F C0 E0            [24] 1223 	push	acc
      000391 12 06 D9         [24] 1224 	lcall	_printf
      000394 E5 81            [12] 1225 	mov	a,sp
      000396 24 FB            [12] 1226 	add	a,#0xfb
      000398 F5 81            [12] 1227 	mov	sp,a
                                   1228 ;	src/main.c:184: }
      00039A 22               [24] 1229 	ret
                                   1230 ;------------------------------------------------------------
                                   1231 ;Allocation info for local variables in function 'main'
                                   1232 ;------------------------------------------------------------
                                   1233 ;	src/main.c:189: void main(void) 
                                   1234 ;	-----------------------------------------
                                   1235 ;	 function main
                                   1236 ;	-----------------------------------------
      00039B                       1237 _main:
                                   1238 ;	src/main.c:192: Set_All_GPIO_Quasi_Mode;
      00039B 75 B1 00         [24] 1239 	mov	_P0M1,#0x00
      00039E 75 B2 00         [24] 1240 	mov	_P0M2,#0x00
      0003A1 75 B3 00         [24] 1241 	mov	_P1M1,#0x00
      0003A4 75 B4 00         [24] 1242 	mov	_P1M2,#0x00
      0003A7 75 AC 00         [24] 1243 	mov	_P3M1,#0x00
      0003AA 75 AD 00         [24] 1244 	mov	_P3M2,#0x00
                                   1245 ;	src/main.c:193: GPIO_Init();
      0003AD 12 00 31         [24] 1246 	lcall	_GPIO_Init
                                   1247 ;	src/main.c:194: UART_Init();
      0003B0 12 01 B3         [24] 1248 	lcall	_UART_Init
                                   1249 ;	src/main.c:197: Timer0_Delay1ms(100);
      0003B3 90 00 64         [24] 1250 	mov	dptr,#(0x64&0x00ff)
      0003B6 E4               [12] 1251 	clr	a
      0003B7 F5 F0            [12] 1252 	mov	b,a
      0003B9 12 05 3B         [24] 1253 	lcall	_Timer0_Delay1ms
                                   1254 ;	src/main.c:200: printf("Line Follower Started\r\n");
      0003BC 74 12            [12] 1255 	mov	a,#___str_9
      0003BE C0 E0            [24] 1256 	push	acc
      0003C0 74 10            [12] 1257 	mov	a,#(___str_9 >> 8)
      0003C2 C0 E0            [24] 1258 	push	acc
      0003C4 74 80            [12] 1259 	mov	a,#0x80
      0003C6 C0 E0            [24] 1260 	push	acc
      0003C8 12 06 D9         [24] 1261 	lcall	_printf
      0003CB 15 81            [12] 1262 	dec	sp
      0003CD 15 81            [12] 1263 	dec	sp
      0003CF 15 81            [12] 1264 	dec	sp
                                   1265 ;	src/main.c:202: while(1)
      0003D1                       1266 00102$:
                                   1267 ;	src/main.c:205: Read_Line_Sensors();
      0003D1 12 01 BD         [24] 1268 	lcall	_Read_Line_Sensors
                                   1269 ;	src/main.c:208: line_pattern = Get_Line_Pattern();
      0003D4 12 02 0B         [24] 1270 	lcall	_Get_Line_Pattern
      0003D7 85 82 13         [24] 1271 	mov	_line_pattern,dpl
      0003DA 85 83 14         [24] 1272 	mov	(_line_pattern + 1),dph
                                   1273 ;	src/main.c:218: Send_JSON_Format();
      0003DD 12 02 B9         [24] 1274 	lcall	_Send_JSON_Format
                                   1275 ;	src/main.c:227: Timer0_Delay1ms(100);  // 50ms = 20Hz update rate
      0003E0 90 00 64         [24] 1276 	mov	dptr,#(0x64&0x00ff)
      0003E3 E4               [12] 1277 	clr	a
      0003E4 F5 F0            [12] 1278 	mov	b,a
      0003E6 12 05 3B         [24] 1279 	lcall	_Timer0_Delay1ms
                                   1280 ;	src/main.c:229: }
      0003E9 80 E6            [24] 1281 	sjmp	00102$
                                   1282 	.area CSEG    (CODE)
                                   1283 	.area CONST   (CODE)
                                   1284 	.area CONST   (CODE)
      000FD8                       1285 ___str_0:
      000FD8 25 75                 1286 	.ascii "%u"
      000FDA 0D                    1287 	.db 0x0d
      000FDB 0A                    1288 	.db 0x0a
      000FDC 00                    1289 	.db 0x00
                                   1290 	.area CSEG    (CODE)
                                   1291 	.area CONST   (CODE)
      000FDD                       1292 ___str_1:
      000FDD 25 75                 1293 	.ascii "%u"
      000FDF 00                    1294 	.db 0x00
                                   1295 	.area CSEG    (CODE)
                                   1296 	.area CONST   (CODE)
      000FE0                       1297 ___str_2:
      000FE0 20                    1298 	.ascii " "
      000FE1 00                    1299 	.db 0x00
                                   1300 	.area CSEG    (CODE)
                                   1301 	.area CONST   (CODE)
      000FE2                       1302 ___str_3:
      000FE2 0D                    1303 	.db 0x0d
      000FE3 0A                    1304 	.db 0x0a
      000FE4 00                    1305 	.db 0x00
                                   1306 	.area CSEG    (CODE)
                                   1307 	.area CONST   (CODE)
      000FE5                       1308 ___str_4:
      000FE5 7B                    1309 	.ascii "{"
      000FE6 22                    1310 	.db 0x22
      000FE7 73 65 6E 73 6F 72 73  1311 	.ascii "sensors"
      000FEE 22                    1312 	.db 0x22
      000FEF 3A 5B                 1313 	.ascii ":["
      000FF1 00                    1314 	.db 0x00
                                   1315 	.area CSEG    (CODE)
                                   1316 	.area CONST   (CODE)
      000FF2                       1317 ___str_5:
      000FF2 2C                    1318 	.ascii ","
      000FF3 00                    1319 	.db 0x00
                                   1320 	.area CSEG    (CODE)
                                   1321 	.area CONST   (CODE)
      000FF4                       1322 ___str_6:
      000FF4 5D 2C                 1323 	.ascii "],"
      000FF6 22                    1324 	.db 0x22
      000FF7 70 61 74 74 65 72 6E  1325 	.ascii "pattern"
      000FFE 22                    1326 	.db 0x22
      000FFF 3A 25 75 7D           1327 	.ascii ":%u}"
      001003 0D                    1328 	.db 0x0d
      001004 0A                    1329 	.db 0x0a
      001005 00                    1330 	.db 0x00
                                   1331 	.area CSEG    (CODE)
                                   1332 	.area CONST   (CODE)
      001006                       1333 ___str_7:
      001006 25 63                 1334 	.ascii "%c"
      001008 00                    1335 	.db 0x00
                                   1336 	.area CSEG    (CODE)
                                   1337 	.area CONST   (CODE)
      001009                       1338 ___str_8:
      001009 4C 3A 25 30 33 58     1339 	.ascii "L:%03X"
      00100F 0D                    1340 	.db 0x0d
      001010 0A                    1341 	.db 0x0a
      001011 00                    1342 	.db 0x00
                                   1343 	.area CSEG    (CODE)
                                   1344 	.area CONST   (CODE)
      001012                       1345 ___str_9:
      001012 4C 69 6E 65 20 46 6F  1346 	.ascii "Line Follower Started"
             6C 6C 6F 77 65 72 20
             53 74 61 72 74 65 64
      001027 0D                    1347 	.db 0x0d
      001028 0A                    1348 	.db 0x0a
      001029 00                    1349 	.db 0x00
                                   1350 	.area CSEG    (CODE)
                                   1351 	.area CABS    (ABS,CODE)
