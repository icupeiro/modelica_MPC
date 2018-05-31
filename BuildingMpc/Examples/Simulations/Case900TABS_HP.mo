within BuildingMpc.Examples.Simulations;
model Case900TABS_HP
  "Controller model for the BESTEST Case900 with TABS and heat pump with an ideal source; the optimization variables are the outlet temperature of the HP and the mass flows through HP cond/evap"
 extends Optimizations.Case900TABS_HP(optVar1(y=mpcCase900TABS_HP.yOpt[1]),
                                      optVar2(y=mpcCase900TABS_HP.yOpt[2]),
                                      optVar3(y=mpcCase900TABS_HP.yOpt[3]));
  MPCs.MpcCase900TABS_HP mpcCase900TABS_HP
    annotation (Placement(transformation(extent={{-84,20},{-64,40}})));
  MPCs.MpcCase900TABS_HP mpcCase900TABS_HP1
    annotation (Placement(transformation(extent={{-84,20},{-64,40}})));
end Case900TABS_HP;
