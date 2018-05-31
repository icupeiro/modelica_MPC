within BuildingMpc.Examples.Simulations;
model Case900TABS
  extends Optimizations.Case900TABS(optVar(y=mpcCase900TABS.yOpt[1]));
  MPCs.MpcCase900TABS mpcCase900TABS
    annotation (Placement(transformation(extent={{-90,-22},{-70,-2}})));
  annotation (experiment(
      StartTime=1420000000,
      StopTime=1430000000,
      __Dymola_fixedstepsize=1,
      __Dymola_Algorithm="Euler"));
end Case900TABS;
