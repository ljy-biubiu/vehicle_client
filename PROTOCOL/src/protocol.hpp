#ifndef PROTOCOL_H
#define PROTOCOL_H

/*recv ring buffer size*/
#define RECEIVE_BUFFER_SIZE 4 * 138

struct ReadData
{
  float angle;
  float amplitude_position;
  float height;
  float safe_distance;
  float resolution;
  float build_map_signal;
  float comnunication_flag;
  float device_status;
  float material_area_x;
  float material_area_y;
  float meterial_area_length;
  float meterial_area_width;
  float technology;
};

struct InCabinArea
{
  float inCabin_A_X;
  float inCabin_A_Y;
  float inCabin_A_Z;
  float inCabin_B_X;
  float inCabin_B_Y;
  float inCabin_B_Z;
  float inCabin_C_X;
  float inCabin_C_Y;
  float inCabin_C_Z;
  float inCabin_D_X;
  float inCabin_D_Y;
  float inCabin_D_Z;
};

struct HatchArea
{
  float hatch_A_X;
  float hatch_A_Y;
  float hatch_A_Z;
  float hatch_B_X;
  float hatch_B_Y;
  float hatch_B_Z;
  float hatch_C_X;
  float hatch_C_Y;
  float hatch_C_Z;
  float hatch_D_X;
  float hatch_D_Y;
  float hatch_D_Z;
};

struct AisleArea
{
  float aisle_A_X;
  float aisle_A_Y;
  float aisle_B_X;
  float aisle_B_Y;
  float aisle_C_X;
  float aisle_C_Y;
  float aisle_D_X;
  float aisle_D_Y;
};

struct DeckArea
{
  float deck_A_X;
  float deck_A_Y;
  float deck_B_X;
  float deck_B_Y;
  float deck_C_X;
  float deck_C_Y;
  float deck_D_X;
  float deck_D_Y;
};

struct MterialArea
{
  float mterial_A_X;
  float mterial_A_Y;
  float mterial_B_X;
  float mterial_B_Y;
  float mterial_C_X;
  float mterial_C_Y;
  float mterial_D_X;
  float mterial_D_Y;
};

struct GrabMaterialArea
{
  float area1_X;
  float area1_Y;
  float area1_Z;
  float area2_X;
  float area2_Y;
  float area2_Z;
  float area3_X;
  float area3_Y;
  float area3_Z;
  float area4_X;
  float area4_Y;
  float area4_Z;
  float area5_X;
  float area5_Y;
  float area5_Z;
  float area6_X;
  float area6_Y;
  float area6_Z;
  float area7_X;
  float area7_Y;
  float area7_Z;
  float area8_X;
  float area8_Y;
  float area8_Z;
  float area9_X;
  float area9_Y;
  float area9_Z;
  float area10_X;
  float area10_Y;
  float area10_Z;

  float area11_X;
  float area11_Y;
  float area11_Z;
  float area12_X;
  float area12_Y;
  float area12_Z;
  float area13_X;
  float area13_Y;
  float area13_Z;
  float area14_X;
  float area14_Y;
  float area14_Z;
  float area15_X;
  float area15_Y;
  float area15_Z;
  float area16_X;
  float area16_Y;
  float area16_Z;
  float area17_X;
  float area17_Y;
  float area17_Z;
  float area18_X;
  float area18_Y;
  float area18_Z;
  float area19_X;
  float area19_Y;
  float area19_Z;
  float area20_X;
  float area20_Y;
  float area20_Z;

  float area21_X;
  float area21_Y;
  float area21_Z;
  float area22_X;
  float area22_Y;
  float area22_Z;
  float area23_X;
  float area23_Y;
  float area23_Z;
  float area24_X;
  float area24_Y;
  float area24_Z;
  float area25_X;
  float area25_Y;
  float area25_Z;
};

struct SetMaterialArea
{
  float area1_X;
  float area1_Y;
  float area1_Z;

  float area2_Z;
  float area3_Z;
  float area4_Z;
  float area5_Z;
  float area6_Z;
  float area7_Z;
  float area8_Z;
  float area9_Z;
};

struct Status
{
  float build_map_signal;
  float comnunication_flag;
  float data_cnt;
  float fault;
};

struct WriteData
{
  InCabinArea inCabinArea;
  HatchArea hatchArea;
  AisleArea aisleArea;
  DeckArea deckArea;
  MterialArea mterialArea;
  GrabMaterialArea grabMaterialArea;
  SetMaterialArea setMaterialArea;
  Status status;
};

#endif
