within BuildingMpc.Examples.Optimizations;
model Case900TABS_2var
  extends ControllerModels.Case900TABS_2var(
                                         optVar1(y=mpcCase900TABS_2var.yOpt[1]),
      optVar2(y=mpcCase900TABS_2var.yOpt[2]));
  MPCs.MpcCase900TABS_2var mpcCase900TABS_2var
    annotation (Placement(transformation(extent={{-92,10},{-72,30}})));
  annotation (
    experiment(
      StopTime=31536000,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=10,
      __Dymola_Algorithm="Euler"),
    __Dymola_experimentSetupOutput,
    __Dymola_experimentFlags(
      Advanced(GenerateVariableDependencies=false, OutputModelicaCode=false),
      Evaluate=false,
      OutputCPUtime=false,
      OutputFlatModelica=false));
end Case900TABS_2var;
