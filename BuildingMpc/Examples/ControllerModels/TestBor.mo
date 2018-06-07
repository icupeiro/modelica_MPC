within BuildingMpc.Examples.ControllerModels;
model TestBor
  extends Case900GEOTABS_1bor(
    optVar3(y=0.5),
    optVar1(y=0.5),
    optVar2(y=273.15 + 30));
  annotation (
    experiment(
      StopTime=31536000,
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
end TestBor;
