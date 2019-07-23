within BuildingMpc.Examples.SimulationModels.Case900Paper;
model factorCalc

  Modelica.Blocks.Sources.Constant const[9](k={1,0.9,0.8,0.7,0.6,0.5,0.4,0.3,
        0.2}) annotation (Placement(transformation(extent={{-60,40},{-40,60}})));
  nomCon nomCon1[9]
    annotation (Placement(transformation(extent={{0,20},{20,40}})));
  nomCon nomCon2 annotation (Placement(transformation(extent={{0,-20},{20,0}})));
  Modelica.Blocks.Sources.Ramp ramp(
    duration=3600,
    startTime=0,
    height=0.9,
    offset=0.1)
    annotation (Placement(transformation(extent={{-60,-20},{-40,0}})));
equation
  connect(const.y, nomCon1.y) annotation (Line(points={{-39,50},{-20,50},{-20,
          33},{0,33}}, color={0,0,127}));
  connect(ramp.y, nomCon2.y) annotation (Line(points={{-39,-10},{-20,-10},{-20,
          -7},{0,-7}}, color={0,0,127}));
end factorCalc;
