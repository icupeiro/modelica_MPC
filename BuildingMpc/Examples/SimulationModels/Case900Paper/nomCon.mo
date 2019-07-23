within BuildingMpc.Examples.SimulationModels.Case900Paper;
model nomCon
  package Water = IDEAS.Media.Water;
  package Glycol = IBPSA.Media.Antifreeze.PropyleneGlycolWater(property_T=281.15, X_a=0.3);

  IDEAS.Fluid.HeatPumps.ScrollWaterToWater heaPum(
    redeclare package Medium1 = IDEAS.Media.Water,
    dp1_nominal=10000,
    dp2_nominal=10000,
    redeclare package ref = IDEAS.Media.Refrigerants.R410A,
    enable_temperature_protection=false,
    redeclare package Medium2 = Glycol,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
  datHeaPum=
      IDEAS.Fluid.HeatPumps.Data.ScrollWaterToWater.Heating.ClimateMaster_TMW036_12kW_4_90COP_R410A(),
    scaling_factor=0.126,
    m1_flow_nominal=0.1,
    m2_flow_nominal=0.1,
    T1_start=298.15,
    T2_start=283.15)                              annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={0,2})));

  IDEAS.Fluid.Sources.Boundary_pT sou(
    nPorts=2,
    redeclare package Medium = Water,
    use_T_in=false,
    p=200000,
    T=278.15)
    annotation (Placement(transformation(extent={{60,40},{40,60}})));
  IDEAS.Fluid.Sources.Boundary_pT sin(
    redeclare package Medium = Water,
    nPorts=2,
    use_T_in=false,
    p=200000,
    T=298.15)
    annotation (Placement(transformation(extent={{-80,-44},{-60,-24}})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow pump_sin(
    redeclare package Medium = Water,
    addPowerToMedium=false,
    inputType=IDEAS.Fluid.Types.InputType.Constant,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    use_inputFilter=false,
    T_start=298.15,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{54,-14},{74,6}})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow pump_sou(
    addPowerToMedium=false,
    inputType=IDEAS.Fluid.Types.InputType.Constant,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    use_inputFilter=false,
    redeclare package Medium = Water,
    T_start=283.15,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{-34,36},{-54,56}})));
  IDEAS.Fluid.Sources.Boundary_pT sin1(redeclare package Medium = Water, nPorts=2)
    annotation (Placement(transformation(extent={{26,-78},{46,-58}})));
  IDEAS.Fluid.Sources.Boundary_pT sou1(nPorts=2, redeclare package Medium =
        Water)
    annotation (Placement(transformation(extent={{60,68},{40,88}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_eva_out(
    redeclare package Medium = Glycol,
    m_flow_nominal=pump_sou.m_flow_nominal,
    tau=0,
    T_start=283.15)
            annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-22,24})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_out(
    redeclare package Medium = Water,
    m_flow_nominal=pump_sin.m_flow_nominal,
    tau=0,
    T_start=298.15)                         annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={32,-4})));
  Modelica.Blocks.Sources.RealExpression COPThe(y=heaPum.QCon_flow/heaPum.com.PThe)
    annotation (Placement(transformation(extent={{-32,-84},{-12,-64}})));
  Fluid.HeatPumps.HeatPump_y
                           heatPump_y(
    redeclare package Medium1 = IDEAS.Media.Water,
    redeclare package Medium2 = Glycol,
    dp1_nominal=0,
    dp2_nominal=0,
    etaCom=1,
    m1_flow_nominal=0.1,
    m2_flow_nominal=0.1,
    PLos=42.1282)
    annotation (Placement(transformation(extent={{-6,-42},{14,-62}})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow pump_sin1(
    redeclare package Medium = Water,
    addPowerToMedium=false,
    inputType=IDEAS.Fluid.Types.InputType.Constant,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    use_inputFilter=false,
    m_flow_nominal=0.1,
    T_start=298.15)
    annotation (Placement(transformation(extent={{30,-46},{50,-26}})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow pump_sou1(
    addPowerToMedium=false,
    inputType=IDEAS.Fluid.Types.InputType.Constant,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    use_inputFilter=false,
    redeclare package Medium = Water,
    T_start=283.15,
    m_flow_nominal=0.1)
    annotation (Placement(transformation(extent={{-40,-20},{-60,0}})));
  Modelica.Blocks.Interfaces.RealInput y(start=1)
    annotation (Placement(transformation(extent={{-120,10},{-80,50}})));
  Modelica.Blocks.Interfaces.RealOutput COP
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealOutput QCon
    annotation (Placement(transformation(extent={{100,-34},{120,-14}})));
equation

  connect(sin.ports[1], heaPum.port_a1) annotation (Line(points={{-60,-32},{-36,
          -32},{-36,-4},{-10,-4}}, color={0,127,255}));
  connect(sou.ports[1], heaPum.port_a2) annotation (Line(points={{40,52},{26,52},
          {26,8},{10,8}}, color={0,127,255}));
  connect(pump_sin.port_b, sin1.ports[1])
    annotation (Line(points={{74,-4},{74,-66},{46,-66}}, color={0,127,255}));
  connect(pump_sou.port_b, sou1.ports[1]) annotation (Line(points={{-54,46},{
          -72,46},{-72,80},{40,80},{40,80}},
                                         color={0,127,255}));
  connect(heaPum.port_b2, T_eva_out.port_a)
    annotation (Line(points={{-10,8},{-22,8},{-22,14}}, color={0,127,255}));
  connect(T_eva_out.port_b, pump_sou.port_a)
    annotation (Line(points={{-22,34},{-22,46},{-34,46}}, color={0,127,255}));
  connect(heaPum.port_b1, T_con_out.port_a)
    annotation (Line(points={{10,-4},{22,-4}}, color={0,127,255}));
  connect(T_con_out.port_b, pump_sin.port_a)
    annotation (Line(points={{42,-4},{54,-4}}, color={0,127,255}));
connect(heatPump_y.port_b1, pump_sin1.port_a) annotation (Line(points={
        {14,-58},{22,-58},{22,-36},{30,-36}}, color={0,127,255}));
  connect(pump_sin1.port_b, sin1.ports[2]) annotation (Line(points={{50,-36},{
          52,-36},{52,-70},{46,-70}}, color={0,127,255}));
connect(sin.ports[2], heatPump_y.port_a1) annotation (Line(points={{-60,
        -36},{-34,-36},{-34,-58},{-6,-58}}, color={0,127,255}));
connect(heatPump_y.port_b2, pump_sou1.port_a) annotation (Line(points={
        {-6,-46},{-24,-46},{-24,-10},{-40,-10}}, color={0,127,255}));
  connect(pump_sou1.port_b, sou1.ports[2]) annotation (Line(points={{-60,-10},{
          -62,-10},{-62,76},{40,76}}, color={0,127,255}));
connect(sou.ports[2], heatPump_y.port_a2) annotation (Line(points={{40,
        48},{28,48},{28,-46},{14,-46}}, color={0,127,255}));
  connect(heaPum.y, y) annotation (Line(points={{-12,-1},{-50,-1},{-50,30},{
          -100,30}}, color={0,0,127}));
  connect(heatPump_y.y, y) annotation (Line(points={{-6,-61},{-50,-61},{-50,30},
          {-100,30}}, color={0,0,127}));
  connect(COPThe.y, COP) annotation (Line(points={{-11,-74},{86,-74},{86,0},{
          110,0}}, color={0,0,127}));
  connect(heaPum.QCon_flow, QCon) annotation (Line(points={{11,-7},{56.5,-7},{
          56.5,-24},{110,-24}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end nomCon;
