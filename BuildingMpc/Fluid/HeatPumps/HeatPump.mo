within BuildingMpc.Fluid.HeatPumps;
model HeatPump "A heat pump model for optimization"
  Modelica.Blocks.Sources.RealExpression COP(y=6.4 - 0.16*(T_con_in.T - 298.15)
         + 0.1*(T_eva.T - 288.15))
    annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
  IDEAS.Fluid.HeatExchangers.PrescribedOutlet HP_con(
    redeclare package Medium = IDEAS.Media.Water,
    allowFlowReversal=false,
    m_flow_nominal=5,
    dp_nominal=0,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    use_TSet=true,
    use_X_wSet=false)
    annotation (Placement(transformation(extent={{-10,50},{10,70}})));
public
  IDEAS.Fluid.HeatExchangers.HeaterCooler_u HP_eva(
    redeclare package Medium = IDEAS.Media.Water,
    allowFlowReversal=false,
    m_flow_nominal=5,
    dp_nominal=0,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    Q_flow_nominal(displayUnit="kW") = 5000)
    annotation (Placement(transformation(extent={{10,-72},{-10,-52}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_eva(
    redeclare package Medium = IDEAS.Media.Water,
    allowFlowReversal=false,
    m_flow_nominal=5,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState)
    annotation (Placement(transformation(extent={{68,-72},{48,-52}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_in(
    redeclare package Medium = IDEAS.Media.Water,
    allowFlowReversal=false,
    m_flow_nominal=5,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState)
    annotation (Placement(transformation(extent={{-66,50},{-46,70}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_out(
    redeclare package Medium = IDEAS.Media.Water,
    allowFlowReversal=false,
    m_flow_nominal=5,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState)
    annotation (Placement(transformation(extent={{50,50},{70,70}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_eva1(
    redeclare package Medium = IDEAS.Media.Water,
    allowFlowReversal=false,
    m_flow_nominal=5,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState)
    annotation (Placement(transformation(extent={{-48,-72},{-68,-52}})));
  Modelica.Blocks.Sources.RealExpression Q_eva(y=-HP_con.Q_flow*COP.y)
    annotation (Placement(transformation(extent={{46,-42},{26,-22}})));
equation
  connect(HP_eva.port_b, T_eva1.port_a)
    annotation (Line(points={{-10,-62},{-48,-62}}, color={0,127,255}));
  connect(HP_eva.port_a, T_eva.port_b)
    annotation (Line(points={{10,-62},{48,-62}}, color={0,127,255}));
  connect(HP_con.port_b, T_con_out.port_a)
    annotation (Line(points={{10,60},{50,60}}, color={0,127,255}));
  connect(T_con_in.port_b, HP_con.port_a)
    annotation (Line(points={{-46,60},{-10,60}}, color={0,127,255}));
  connect(Q_eva.y, HP_eva.u) annotation (Line(points={{25,-32},{25,-56},{20,-56},
          {20,-56},{12,-56},{12,-56}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end HeatPump;
