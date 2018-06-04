within BuildingMpc.Examples.Optimizations;
model Case900TABS
  extends ControllerModels.Case900TABS(
                                    optVar(y=mpcCase900TABS.yOpt[1]));
  MPCs.MpcCase900TABS mpcCase900TABS
    annotation (Placement(transformation(extent={{-90,-22},{-70,-2}})));
  annotation (experiment(
      StopTime=31536000,
      __Dymola_NumberOfIntervals=15000,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=10,
      __Dymola_Algorithm="Euler"));
end Case900TABS;
