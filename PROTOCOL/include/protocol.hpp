#ifndef PROTOCOL_H
#define PROTOCOL_H

/*recv ring buffer size*/
#define RECEIVE_BUFFER_SIZE 138 * 4
#define RECEIVE_DATA_SIZE 138

struct ReadData
{
  float angle{0};
  float amplitude_position{0};
  float height{0};
  float safe_distance{0};
  float resolution{0};
  float build_map_signal{0};
  float comnunication_flag{0};
  float device_status{0};
  float material_area_x{0};
  float material_area_y{0};
  float meterial_area_length{0};
  float meterial_area_width{0};
  float technology{0};
};

struct InCabinArea
{
  float inCabin_A_X{0};
  float inCabin_A_Y{0};
  float inCabin_A_Z{0};
  float inCabin_B_X{0};
  float inCabin_B_Y{0};
  float inCabin_B_Z{0};
  float inCabin_C_X{0};
  float inCabin_C_Y{0};
  float inCabin_C_Z{0};
  float inCabin_D_X{0};
  float inCabin_D_Y{0};
  float inCabin_D_Z{0};
};

struct HatchArea
{
  float hatch_A_X{0};
  float hatch_A_Y{0};
  float hatch_A_Z{0};
  float hatch_B_X{0};
  float hatch_B_Y{0};
  float hatch_B_Z{0};
  float hatch_C_X{0};
  float hatch_C_Y{0};
  float hatch_C_Z{0};
  float hatch_D_X{0};
  float hatch_D_Y{0};
  float hatch_D_Z{0};
};

struct AisleArea
{
  float aisle_A_X{0};
  float aisle_A_Y{0};
  float aisle_B_X{0};
  float aisle_B_Y{0};
  float aisle_C_X{0};
  float aisle_C_Y{0};
  float aisle_D_X{0};
  float aisle_D_Y{0};
};

struct DeckArea
{
  float deck_A_X{0};
  float deck_A_Y{0};
  float deck_B_X{0};
  float deck_B_Y{0};
  float deck_C_X{0};
  float deck_C_Y{0};
  float deck_D_X{0};
  float deck_D_Y{0};
};

struct MterialArea
{
  float mterial_A_X{0};
  float mterial_A_Y{0};
  float mterial_B_X{0};
  float mterial_B_Y{0};
  float mterial_C_X{0};
  float mterial_C_Y{0};
  float mterial_D_X{0};
  float mterial_D_Y{0};
};

struct GrabMaterialArea
{
  float area1_X{0};
  float area1_Y{0};
  float area1_Z{0};
  float area2_X{0};
  float area2_Y{0};
  float area2_Z{0};
  float area3_X{0};
  float area3_Y{0};
  float area3_Z{0};
  float area4_X{0};
  float area4_Y{0};
  float area4_Z{0};
  float area5_X{0};
  float area5_Y{0};
  float area5_Z{0};
  float area6_X{0};
  float area6_Y{0};
  float area6_Z{0};
  float area7_X{0};
  float area7_Y{0};
  float area7_Z{0};
  float area8_X{0};
  float area8_Y{0};
  float area8_Z{0};
  float area9_X{0};
  float area9_Y{0};
  float area9_Z{0};
  float area10_X{0};
  float area10_Y{0};
  float area10_Z{0};

  float area11_X{0};
  float area11_Y{0};
  float area11_Z{0};
  float area12_X{0};
  float area12_Y{0};
  float area12_Z{0};
  float area13_X{0};
  float area13_Y{0};
  float area13_Z{0};
  float area14_X{0};
  float area14_Y{0};
  float area14_Z{0};
  float area15_X{0};
  float area15_Y{0};
  float area15_Z{0};
  float area16_X{0};
  float area16_Y{0};
  float area16_Z{0};
  float area17_X{0};
  float area17_Y{0};
  float area17_Z{0};
  float area18_X{0};
  float area18_Y{0};
  float area18_Z{0};
  float area19_X{0};
  float area19_Y{0};
  float area19_Z{0};
  float area20_X{0};
  float area20_Y{0};
  float area20_Z{0};

  float area21_X{0};
  float area21_Y{0};
  float area21_Z{0};
  float area22_X{0};
  float area22_Y{0};
  float area22_Z{0};
  float area23_X{0};
  float area23_Y{0};
  float area23_Z{0};
  float area24_X{0};
  float area24_Y{0};
  float area24_Z{0};
  float area25_X{0};
  float area25_Y{0};
  float area25_Z{0};
};

struct SetMaterialArea
{
  float area1_X{0};
  float area1_Y{0};
  float area1_Z{0};

  float area2_Z{0};
  float area3_Z{0};
  float area4_Z{0};
  float area5_Z{0};
  float area6_Z{0};
  float area7_Z{0};
  float area8_Z{0};
  float area9_Z{0};
};

struct Status
{
  float build_map_signal{0};
  float comnunication_flag{0};
  float data_cnt{0};
  float fault{0};
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

struct ComunicationData
{
    ReadData read_data;
    WriteData write_data;
};

#endif
