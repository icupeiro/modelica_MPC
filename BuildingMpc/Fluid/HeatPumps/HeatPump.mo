within BuildingMpc.Fluid.HeatPumps;
model HeatPump "A heat pump model for optimization"

  replaceable package Medium1 =
      Modelica.Media.Interfaces.PartialMedium "Medium through the sink side (condenser)"
      annotation (choicesAllMatching = true);

          replaceable package Medium2 = Modelica.Media.Interfaces.PartialMedium
    "Medium through the source side (evaporator)" annotation (
      choicesAllMatching=true);

  Modelica.Blocks.Sources.RealExpression COP(y=COP_expr)
    "COP expression of the heat pump"
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  IDEAS.Fluid.HeatExchangers.PrescribedOutlet HP_con(
    allowFlowReversal=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    use_TSet=true,
    use_X_wSet=false,
    redeclare package Medium = Medium1,
    m_flow_nominal=m1_flow_nominal,
    dp_nominal=dp1_nominal,
    tau=0)
    annotation (Placement(transformation(extent={{-10,50},{10,70}})));
public
  IDEAS.Fluid.HeatExchangers.HeaterCooler_u HP_eva(
    allowFlowReversal=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    Q_flow_nominal(displayUnit="W") = 1,
    m_flow_nominal=m2_flow_nominal,
    dp_nominal=dp2_nominal,
    redeclare package Medium = Medium2)
    annotation (Placement(transformation(extent={{10,-70},{-10,-50}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_eva_in(
    allowFlowReversal=false,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    redeclare package Medium = Medium2,
    m_flow_nominal=m2_flow_nominal)
    annotation (Placement(transformation(extent={{70,-70},{50,-50}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_in(
    allowFlowReversal=false,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    redeclare package Medium = Medium1,
    m_flow_nominal=m1_flow_nominal)
    annotation (Placement(transformation(extent={{-70,50},{-50,70}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_out(
    allowFlowReversal=false,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    redeclare package Medium = Medium1,
    m_flow_nominal=m1_flow_nominal)
    annotation (Placement(transformation(extent={{50,50},{70,70}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_eva_out(
    allowFlowReversal=false,
    tau=0,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    redeclare package Medium = Medium2,
    m_flow_nominal=m2_flow_nominal)
    annotation (Placement(transformation(extent={{-50,-70},{-70,-50}})));
  Modelica.Blocks.Sources.RealExpression Q_eva(y=-HP_con.Q_flow*COP.y)
    annotation (Placement(transformation(extent={{50,-40},{30,-20}})));

  parameter Modelica.SIunits.MassFlowRate m1_flow_nominal
    "Nominal mass flow rate through the condenser"
    annotation (Dialog(group="Nominal conditions"));
  parameter Modelica.SIunits.PressureDifference dp1_nominal
    "Pressure difference through the condenser"
    annotation (Dialog(group="Nominal conditions"));
  parameter Modelica.SIunits.MassFlowRate m2_flow_nominal
    "Nominal mass flow rate through the evaporator"
    annotation (Dialog(group="Nominal conditions"));
  parameter Modelica.SIunits.PressureDifference dp2_nominal
    "Pressure difference through the evaporator"
    annotation (Dialog(group="Nominal conditions"));

  Modelica.Blocks.Interfaces.RealOutput COP_expr=6.4 - 0.16*(T_con_in.T - 298.15)
       + 0.1*(T_eva_in.T - 288.15) "COP expression of the heat pump"
    annotation (Dialog(tab="Advanced"));
  Modelica.Blocks.Interfaces.RealInput Tcon_out
    "outlet condenser temperature signal"
    annotation (Placement(transformation(extent={{-120,70},{-80,110}})));
  Modelica.Blocks.Sources.RealExpression Wcomp(y=HP_con.Q_flow/COP.y)
    "compressor power"
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  Modelica.Blocks.Interfaces.RealOutput W_comp
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a1(redeclare package Medium =
        Medium1)
    annotation (Placement(transformation(extent={{-110,50},{-90,70}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b1(redeclare package Medium =
        Medium1)
    annotation (Placement(transformation(extent={{90,50},{110,70}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a2(redeclare package Medium =
        Medium2)
    annotation (Placement(transformation(extent={{90,-70},{110,-50}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b2(redeclare package Medium =
        Medium2)
    annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
equation
  connect(HP_eva.port_b, T_eva_out.port_a)
    annotation (Line(points={{-10,-60},{-50,-60}}, color={0,127,255}));
  connect(HP_eva.port_a, T_eva_in.port_b)
    annotation (Line(points={{10,-60},{50,-60}}, color={0,127,255}));
  connect(HP_con.port_b, T_con_out.port_a)
    annotation (Line(points={{10,60},{50,60}}, color={0,127,255}));
  connect(T_con_in.port_b, HP_con.port_a)
    annotation (Line(points={{-50,60},{-10,60}}, color={0,127,255}));
  connect(Q_eva.y, HP_eva.u) annotation (Line(points={{29,-30},{29,-30},{24,-30},
          {20,-30},{20,-54},{14,-54},{12,-54}},
                                       color={0,0,127}));
  connect(Tcon_out, HP_con.TSet) annotation (Line(points={{-100,90},{-40,90},{-40,
          68},{-12,68}}, color={0,0,127}));
  connect(Wcomp.y, W_comp)
    annotation (Line(points={{61,0},{110,0}}, color={0,0,127}));
  connect(port_a1, T_con_in.port_a)
    annotation (Line(points={{-100,60},{-70,60}}, color={0,127,255}));
  connect(port_b1, T_con_out.port_b)
    annotation (Line(points={{100,60},{70,60}}, color={0,127,255}));
  connect(port_a2, T_eva_in.port_a)
    annotation (Line(points={{100,-60},{85,-60},{70,-60}}, color={0,127,255}));
  connect(T_eva_out.port_b, port_b2)
    annotation (Line(points={{-70,-60},{-100,-60}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-64,80},{76,-80}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-50,68},{64,50}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-50,-52},{64,-70}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-97,64},{104,54}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{4,54},{104,64}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={255,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-95,-56},{106,-66}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-94,-66},{6,-56}},
          lineColor={0,0,127},
          pattern=LinePattern.None,
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-36,0},{-46,-12},{-26,-12},{-36,0}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{-36,0},{-46,10},{-26,10},{-36,0}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-38,50},{-34,10}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-38,-12},{-34,-52}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{44,50},{48,-52}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{24,22},{68,-20}},
          lineColor={0,0,0},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{46,22},{28,-10},{64,-10},{46,22}},
          lineColor={0,0,0},
          smooth=Smooth.None,
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Line(points={{100,0},{68,0}}, color={28,108,200}),
        Line(points={{-82,90},{80,90},{80,64}}, color={28,108,200}),
                                 Text(
          extent={{-141,157},{159,117}},
          lineColor={0,0,255},
          fillPattern=FillPattern.HorizontalCylinder,
          fillColor={0,127,255},
          textString="%name")}),                                 Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end HeatPump;
