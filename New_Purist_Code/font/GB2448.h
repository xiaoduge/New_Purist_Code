#ifndef __GB2448_H__
#define __GB2448_H__


typedef struct  typFNT_GB49                           // ������ģ���ݽṹ 
{
       unsigned char  Index[3];               // ������������	
       unsigned char   Msk[144];               // ���������� 
};
 struct  typFNT_GB49 codeGB_49[] PROGMEM =          // ���ݱ� 
{
 //  ���� 36������  ��X�ߣ�24X48 
{{"S"},{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x80,0x03,0xFF,0xC0,0x07,0xFF,
0xE0,0x0F,0xE7,0xF0,0x0F,0x81,0xF8,0x1F,0x00,0xF8,0x1F,0x00,0xF8,0x1E,0x00,0x78,
0x1E,0x00,0x78,0x1F,0x00,0x00,0x1F,0x00,0x00,0x1F,0xC0,0x00,0x0F,0xF0,0x00,0x07,
0xFC,0x00,0x03,0xFF,0x00,0x01,0xFF,0xC0,0x00,0x7F,0xE0,0x00,0x0F,0xF0,0x00,0x03,
0xF8,0x00,0x01,0xFC,0x00,0x00,0xFC,0x3C,0x00,0x7C,0x3C,0x00,0x7C,0x3C,0x00,0x3C,
0x3E,0x00,0x3C,0x3E,0x00,0x7C,0x3F,0x00,0x7C,0x1F,0x80,0xF8,0x0F,0xE7,0xF8,0x0F,
0xFF,0xF0,0x07,0xFF,0xE0,0x01,0xFF,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{{"P"},{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,0xFF,0xC0,0x1F,0xFF,0xF0,0x1F,0xFF,
0xF8,0x1E,0x03,0xFC,0x1E,0x00,0x7C,0x1E,0x00,0x3E,0x1E,0x00,0x3E,0x1E,0x00,0x3E,
0x1E,0x00,0x1E,0x1E,0x00,0x1E,0x1E,0x00,0x1E,0x1E,0x00,0x3E,0x1E,0x00,0x3E,0x1E,
0x00,0x3E,0x1E,0x00,0x7C,0x1E,0x03,0xFC,0x1F,0xFF,0xF8,0x1F,0xFF,0xF0,0x1F,0xFF,
0xC0,0x1E,0x00,0x00,0x1E,0x00,0x00,0x1E,0x00,0x00,0x1E,0x00,0x00,0x1E,0x00,0x00,
0x1E,0x00,0x00,0x1E,0x00,0x00,0x1E,0x00,0x00,0x1E,0x00,0x00,0x1E,0x00,0x00,0x1E,
0x00,0x00,0x1E,0x00,0x00,0x1E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},


{{"W"},{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF0,0x7C,0x1E,0xF8,0x7C,0x3E,0xF8,0x7C,
0x3E,0xF8,0xFE,0x3E,0xF8,0xFE,0x3E,0xF8,0xFE,0x3E,0xF8,0xFE,0x3E,0x78,0xFE,0x3C,
0x78,0xFE,0x3C,0x78,0xFE,0x3C,0x79,0xEF,0x3C,0x7D,0xEF,0x7C,0x7D,0xEF,0x7C,0x7D,
0xEF,0x7C,0x7D,0xEF,0x7C,0x3D,0xEF,0x78,0x3D,0xEF,0x78,0x3F,0xC7,0xF8,0x3F,0xC7,
0xF8,0x3F,0xC7,0xF8,0x3F,0xC7,0xF8,0x3F,0xC7,0xF8,0x3F,0xC7,0xF8,0x1F,0xC7,0xF0,
0x1F,0x83,0xF0,0x1F,0x83,0xF0,0x1F,0x83,0xF0,0x1F,0x83,0xF0,0x1F,0x83,0xF0,0x1F,
0x83,0xF0,0x1F,0x83,0xF0,0x0F,0x01,0xE0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{{"-"},{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x7F,0xFF,0xFC,0x7F,0xFF,0xFC,0x7F,0xFF,0xFC,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{{":"},{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x00,0x00,0x3C,0x00,0x00,
0x3C,0x00,0x00,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0x00,0x00,
0x3C,0x00,0x00,0x3C,0x00,0x00,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

};

#endif