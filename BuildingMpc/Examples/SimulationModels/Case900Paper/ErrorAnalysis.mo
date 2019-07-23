within BuildingMpc.Examples.SimulationModels.Case900Paper;
model ErrorAnalysis

  BuildingMpc.Examples.SimulationModels.Case900Paper.COPderivationConMod
    cOPderivationConMod[9]
    annotation (Placement(transformation(extent={{22,26},{42,46}})));
  Modelica.Blocks.Sources.Constant const[9](k={1,0.9,0.8,0.7,0.6,0.5,0.4,0.3,
        0.2}) annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
  COPderivationConModmFlow cOPderivationConModmFlow[9]
    annotation (Placement(transformation(extent={{20,-20},{40,0}})));
equation
  connect(const.y, cOPderivationConMod.y) annotation (Line(points={{-39,50},{
          -10,50},{-10,39},{22,39}}, color={0,0,127}));
  connect(cOPderivationConModmFlow.y, const.y) annotation (Line(points={{20.1,
          -7.1},{-9.95,-7.1},{-9.95,50},{-39,50}}, color={0,0,127}));
end ErrorAnalysis;
