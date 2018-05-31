within BuildingMpc.Examples.Data;
record TradDesignTABS
  "traditional design of TABS for BESTEST example following standards EN1264, 11855 and UPONOR guidelines"
  extends IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.RadiantSlabChar(
  tabs=false,
  T=0.15,
  d_a= 0.016,
  s_r = 0.002,
  S_1=0.05*1.13,
  S_2=0.08-S_1,
  c_b = 1000,
  rho_b=1400);  //Rb is 0.05 (m2K)/W
end TradDesignTABS;
