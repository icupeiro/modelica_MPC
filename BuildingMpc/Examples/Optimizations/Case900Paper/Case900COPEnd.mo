within BuildingMpc.Examples.Optimizations.Case900Paper;
model Case900COPEnd
  extends Case900COP(borFie(TExt0_start=278.15));
  annotation (
    experiment(
      StopTime=5259487,
      Interval=300,
      Tolerance=1e-06,
      __Dymola_fixedstepsize=30,
      __Dymola_Algorithm="Euler"),
    __Dymola_experimentSetupOutput,
    __Dymola_experimentFlags(Advanced(
        InlineMethod=0,
        InlineOrder=2,
        InlineFixedStep=0.001)));
end Case900COPEnd;
