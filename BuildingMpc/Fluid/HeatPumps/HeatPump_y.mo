within BuildingMpc.Fluid.HeatPumps;
model HeatPump_y "A heat pump model for optimization"

  replaceable package Medium1 =
      Modelica.Media.Interfaces.PartialMedium "Medium through the sink side (condenser)"
      annotation (choicesAllMatching = true);

          replaceable package Medium2 = Modelica.Media.Interfaces.PartialMedium
    "Medium through the source side (evaporator)" annotation (
      choicesAllMatching=true);

  Modelica.Blocks.Sources.RealExpression COPthe(y=COP_expr)
    "Theoretical COP expression of the heat pump"
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  IDEAS.Fluid.HeatExchangers.HeaterCooler_u   HP_con(
    allowFlowReversal=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    redeclare package Medium = Medium1,
    m_flow_nominal=m1_flow_nominal,
    dp_nominal=dp1_nominal,
    Q_flow_nominal=Q_con_max)
    annotation (Placement(transformation(extent={{-10,50},{10,70}})));

  IDEAS.Fluid.HeatExchangers.HeaterCooler_u HP_eva(
    allowFlowReversal=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    Q_flow_nominal(displayUnit="W") = 1,
    m_flow_nominal=m2_flow_nominal,
    dp_nominal=dp2_nominal,
    redeclare package Medium = Medium2,
    tau=0)
    annotation (Placement(transformation(extent={{10,-70},{-10,-50}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_eva_in(
    allowFlowReversal=false,
    redeclare package Medium = Medium2,
    m_flow_nominal=m2_flow_nominal,
    tau=0)
    annotation (Placement(transformation(extent={{70,-70},{50,-50}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_in(
    allowFlowReversal=false,
    redeclare package Medium = Medium1,
    m_flow_nominal=m1_flow_nominal,
    tau=0)
    annotation (Placement(transformation(extent={{-70,50},{-50,70}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_out(
    allowFlowReversal=false,
    initType=Modelica.Blocks.Types.Init.SteadyState,
    redeclare package Medium = Medium1,
    m_flow_nominal=m1_flow_nominal,
    tau=0)
    annotation (Placement(transformation(extent={{50,50},{70,70}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_eva_out(
    allowFlowReversal=false,
    redeclare package Medium = Medium2,
    m_flow_nominal=m2_flow_nominal,
    tau=0)
    annotation (Placement(transformation(extent={{-50,-70},{-70,-50}})));
  Modelica.Blocks.Sources.RealExpression Q_eva(y=-(HP_con.Q_flow - Wcomp.y))
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

  parameter Modelica.SIunits.Power PLos = 0
  "Constant term of compressor losses"
  annotation (Dialog(tab="Advanced"));

    parameter Real etaCom = 1
    "Compressor electromechanical efficiency"
    annotation (Dialog(tab="Advanced"));

  Modelica.Blocks.Interfaces.RealOutput COP_expr=
    (7.89 - 0.198*(T_con_in.T - 298.15) + 0.158*(T_eva_in.T - 278.15))*loadFactor
   "Theoretical COP expression of the heat pump"
    annotation (Dialog(tab="Advanced"));
   // (5.44-0.113*(T_con_in.T - 298.15) + 0.114*(T_eva_in.T - 288.15))*loadFactor

  Real COP = HP_con.Q_flow/Wcomp.y
  "Real COP";

  parameter Modelica.SIunits.HeatFlowRate Q_con_max = 885.86
  "Maximum heat capacity of the HP"
  annotation (Dialog(tab="Advanced"));                       //+ 27.698*(T_eva_in.T-273.15) - 2.532*(T_con_in.T - 303.15)

  Real loadFactor = 1.2684 - 0.2841*y;

  Modelica.Blocks.Interfaces.RealInput y "Outlet condenser temperature signal"
    annotation (Placement(transformation(extent={{-120,70},{-80,110}})));

  Modelica.Blocks.Sources.RealExpression Wcomp(y=HP_con.Q_flow/COPthe.y/etaCom + PLos)
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
  parameter Modelica.SIunits.HeatFlowRate Q_nom=Modelica.Constants.inf
    "Heat pump nominal power (heating)"
    annotation (Dialog(group="Nominal conditions"));
  Modelica.Blocks.Interfaces.RealOutput Q_con
    annotation (Placement(transformation(extent={{100,74},{120,94}})));
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
  connect(HP_con.Q_flow, Q_con) annotation (Line(points={{11,66},{12,66},{12,84},
          {110,84}}, color={0,0,127}));
  connect(y, HP_con.u) annotation (Line(points={{-100,90},{-32,90},{-32,66},{-12,
          66}}, color={0,0,127}));
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
end HeatPump_y;
