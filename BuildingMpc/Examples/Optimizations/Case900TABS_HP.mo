within BuildingMpc.Examples.Optimizations;
model Case900TABS_HP
  "Controller model for the BESTEST Case900 with TABS and heat pump with an ideal source; the optimization variables are the outlet temperature of the HP and the mass flows through HP cond/evap"
 extends ControllerModels.Case900TABS_HP(
                                      optVar1(y=mpcCase900TABS_HP.yOpt[1]),
                                      optVar2(y=mpcCase900TABS_HP.yOpt[2]),
                                      optVar3(y=mpcCase900TABS_HP.yOpt[3]));
  MPCs.MpcCase900TABS_HP mpcCase900TABS_HP
    annotation (Placement(transformation(extent={{-84,20},{-64,40}})));
  annotation (experiment(
      StopTime=31536000,
      __Dymola_NumberOfIntervals=15000,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=10,
      __Dymola_Algorithm="Euler"));
end Case900TABS_HP;
