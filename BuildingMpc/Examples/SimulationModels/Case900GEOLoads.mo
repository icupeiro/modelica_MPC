within BuildingMpc.Examples.SimulationModels;
model Case900GEOLoads
  "model to determine the geothermal load to design borefield"
  extends IDEAS.Buildings.Validation.Cases.Case900(redeclare
      BuildingMpc.Examples.SimulationModels.Envelope.Bui900TABS building,
      redeclare HeatingSystems.Hea900IdealGEO heatingSystem);
  Modelica.Blocks.Continuous.Integrator heaEnergy
    annotation (Placement(transformation(extent={{-30,60},{-10,80}})));
  Modelica.Blocks.Continuous.Integrator cooEnergy(k=1)
    annotation (Placement(transformation(extent={{40,60},{60,80}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=if building.heatPortEmb[
        1].Q_flow > 0 then building.heatPortEmb[1].Q_flow else 0)
    annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=if building.heatPortEmb[
        1].Q_flow < 0 then -building.heatPortEmb[1].Q_flow else 0)
    annotation (Placement(transformation(extent={{6,60},{26,80}})));
equation
  connect(realExpression.y, heaEnergy.u)
    annotation (Line(points={{-39,70},{-32,70}}, color={0,0,127}));
  connect(realExpression1.y, cooEnergy.u)
    annotation (Line(points={{27,70},{38,70}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
    experiment(
      StopTime=3.1536e+007,
      __Dymola_NumberOfIntervals=15000,
      Tolerance=1e-006,
      __Dymola_fixedstepsize=10,
      __Dymola_Algorithm="Euler"),
    __Dymola_experimentSetupOutput,
    __Dymola_experimentFlags(Advanced(
        InlineMethod=0,
        InlineOrder=2,
        InlineFixedStep=0.001)));
end Case900GEOLoads;
