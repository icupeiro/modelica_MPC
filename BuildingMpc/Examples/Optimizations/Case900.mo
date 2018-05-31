within BuildingMpc.Examples.Optimizations;
model Case900
  extends ControllerModels.Case900;
  MPCs.MpcCase900 mpcCase900_1
    annotation (Placement(transformation(extent={{-92,-20},{-72,0}})));
  annotation (
    experiment(
      StartTime=1421000000,
      StopTime=1421100000,
      __Dymola_NumberOfIntervals=15000,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=10,
      __Dymola_Algorithm="Euler"),
    __Dymola_experimentSetupOutput,
    __Dymola_experimentFlags(
      Advanced(GenerateVariableDependencies=false, OutputModelicaCode=false),
      Evaluate=false,
      OutputCPUtime=false,
      OutputFlatModelica=false));
end Case900;
