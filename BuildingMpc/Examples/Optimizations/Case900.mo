within BuildingMpc.Examples.Optimizations;
model Case900
  extends ControllerModels.Case900(optVar(y=mpcCase900_1.yOpt[1]));
  MPCs.MpcCase900 mpcCase900_1
    annotation (Placement(transformation(extent={{-92,-20},{-72,0}})));
  annotation (
    experiment(
      StopTime=31536000,
      __Dymola_NumberOfIntervals=15000,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=10,
      __Dymola_Algorithm="Euler"),
    __Dymola_experimentSetupOutput,
    __Dymola_experimentFlags(Advanced(
        InlineMethod=0,
        InlineOrder=2,
        InlineFixedStep=0.001)));
end Case900;
