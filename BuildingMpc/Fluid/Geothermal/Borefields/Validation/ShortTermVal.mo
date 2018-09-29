within BuildingMpc.Fluid.Geothermal.Borefields.Validation;
model ShortTermVal "Validation of the short term model"
  extends Modelica.Icons.Example;
  package Medium = IBPSA.Media.Water;
  parameter Integer nSeg = borHol1.nSeg;

  parameter IBPSA.Fluid.Geothermal.Borefields.Data.Borefield.Example  borFieDat "Borefield data"
    annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
              IBPSA.Fluid.Geothermal.Borefields.BaseClasses.Boreholes.OneUTube                    borHol1(
    redeclare package Medium = Medium,
    borFieDat=borFieDat,
    dp_nominal=borFieDat.conDat.dp_nominal,
    allowFlowReversal=false,
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
    m_flow_small=0.0001,
    computeFlowResistance=false,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    TGro_start=(273.15 + 13.5)*ones(nSeg),
    TFlu_start=(273.15 + 13.5)*ones(nSeg))
    "Borehole connected to a discrete ground model" annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={50,0})));
  IBPSA.Fluid.Sources.Boundary_pT      sou(
    redeclare package Medium = Medium,
    nPorts=1,
    use_T_in=true,
    T=277.15) "Source" annotation (Placement(transformation(extent={{-98,42},{-78,
            62}},     rotation=0)));
  IBPSA.Fluid.Sources.Boundary_pT sin(
    redeclare package Medium = Medium,
    use_p_in=false,
    use_T_in=false,
    nPorts=1) "Sink" annotation (Placement(transformation(extent={{40,-32},{20,
            -12}},rotation=0)));
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorIn(
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
    redeclare package Medium = Medium,
    tau=0) "Inlet borehole temperature"
    annotation (Placement(transformation(extent={{-72,42},{-52,62}})));
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorOut(
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
    redeclare package Medium = Medium,
    tau=0) "Outlet borehole temperature"
    annotation (Placement(transformation(extent={{-32,-30},{-12,-10}})));
  IBPSA.Fluid.Geothermal.Borefields.BaseClasses.HeatTransfer.Cylindrical
    lay1
       [nSeg](
    each r_b=3,
    each soiDat=borFieDat.soiDat,
    each h=borFieDat.conDat.hBor/nSeg,
    each r_a=borFieDat.conDat.rBor,
    each steadyStateInitial=false,
    TInt_start=273.15 + 13.5,
    TExt_start=273.15 + 13.5)                           annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={50,38})));
  Modelica.Thermal.HeatTransfer.Components.HeatCapacitor[nSeg] Cground1(C=
        Modelica.Constants.inf, T(start=273.15 + 13.5, fixed=true))
    annotation (Placement(transformation(extent={{38,70},{62,94}})));
  OneUTube oneUTube(
    borFieDat=borFieDat,
    allowFlowReversal=false,
    redeclare package Medium = Medium,
    Tsoil=273.15 + 13.5,
    m_flow_small=0.0001,
    computeFlowResistance=false,
    TGro_start=(273.15 + 13.5)*ones(nSeg),
    TFlu_start=(273.15 + 13.5)*ones(nSeg))
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorIn1(
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
    redeclare package Medium = Medium,
    tau=0) "Inlet borehole temperature"
    annotation (Placement(transformation(extent={{-6,60},{14,80}})));
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorOut1(
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
    redeclare package Medium = Medium,
    tau=0) "Outlet borehole temperature"
    annotation (Placement(transformation(extent={{72,-10},{92,10}})));
  IBPSA.Fluid.Sources.Boundary_pT sin1(
    redeclare package Medium = Medium,
    use_p_in=false,
    use_T_in=false,
    nPorts=1) "Sink" annotation (Placement(transformation(extent={{146,-10},{
            126,10}},
                  rotation=0)));
  IBPSA.Fluid.Sources.Boundary_pT      sou1(
    redeclare package Medium = Medium,
    nPorts=1,
    use_T_in=true,
    T=277.15) "Source" annotation (Placement(transformation(extent={{-32,60},{
            -12,80}}, rotation=0)));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource mFlow(
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04,
    redeclare final package Medium = Medium,
    control_dp=false) "Pressure source"
    annotation (Placement(transformation(extent={{-6,-30},{14,-10}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource mFlow1(
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04,
    redeclare final package Medium = Medium,
    control_dp=false) "Pressure source"
    annotation (Placement(transformation(extent={{98,-10},{118,10}})));
public
  Modelica.Blocks.Sources.Constant const(k=borFieDat.conDat.mBor_flow_nominal)
    annotation (Placement(transformation(extent={{-140,6},{-120,26}})));
  Modelica.Blocks.Math.Gain gain(k=borFieDat.conDat.nBor)
    annotation (Placement(transformation(extent={{-86,-32},{-66,-12}})));
  Modelica.Blocks.Sources.Sine sine1(
    amplitude=273.15 + 1.5,
    freqHz=1/500,
    offset=273.15 + 9.0)
    annotation (Placement(transformation(extent={{-156,48},{-136,68}})));
  Modelica.Blocks.Math.Add err(k1=-1)
    annotation (Placement(transformation(extent={{42,-82},{62,-62}})));
equation
  connect(sou.ports[1],TBorIn. port_a)
    annotation (Line(points={{-78,52},{-72,52}},
                                               color={0,127,255}));
  connect(borHol1.port_wall, lay1.port_a)
    annotation (Line(points={{50,10},{50,28}}, color={191,0,0}));
  connect(lay1.port_b, Cground1.port)
    annotation (Line(points={{50,48},{50,70}}, color={191,0,0}));
  connect(TBorIn1.port_b, borHol1.port_a) annotation (Line(points={{14,70},{26,70},
          {26,0},{40,0}}, color={0,127,255}));
  connect(TBorIn.port_b, oneUTube.port_a) annotation (Line(points={{-52,52},{-36,
          52},{-36,18},{-82,18},{-82,-2},{-60,-2},{-60,0}}, color={0,127,255}));
  connect(oneUTube.port_b, TBorOut.port_a) annotation (Line(points={{-40,0},{-36,
          0},{-36,-20},{-32,-20}}, color={0,127,255}));
  connect(borHol1.port_b, TBorOut1.port_a)
    annotation (Line(points={{60,0},{72,0}}, color={0,127,255}));
  connect(sou1.ports[1], TBorIn1.port_a)
    annotation (Line(points={{-12,70},{-6,70}}, color={0,127,255}));
  connect(TBorOut.port_b, mFlow.port_a)
    annotation (Line(points={{-12,-20},{-6,-20}}, color={0,127,255}));
  connect(mFlow.port_b, sin.ports[1]) annotation (Line(points={{14,-20},{18,-20},
          {18,-22},{20,-22}}, color={0,127,255}));
  connect(TBorOut1.port_b, mFlow1.port_a)
    annotation (Line(points={{92,0},{98,0}}, color={0,127,255}));
  connect(mFlow1.port_b, sin1.ports[1])
    annotation (Line(points={{118,0},{126,0}}, color={0,127,255}));
  connect(gain.y, mFlow.m_flow_in) annotation (Line(points={{-65,-22},{-34,-22},
          {-34,-12},{-2,-12}}, color={0,0,127}));
  connect(sine1.y, sou1.T_in) annotation (Line(points={{-135,58},{-84,58},{-84,
          74},{-34,74}}, color={0,0,127}));
  connect(sine1.y, sou.T_in) annotation (Line(points={{-135,58},{-118,58},{-118,
          56},{-100,56}}, color={0,0,127}));
  connect(TBorOut.T, err.u2) annotation (Line(points={{-22,-9},{-18,-9},{-18,
          -82},{40,-82},{40,-78}}, color={0,0,127}));
  connect(TBorOut1.T, err.u1) annotation (Line(points={{82,11},{80,11},{80,-46},
          {18,-46},{18,-66},{40,-66}}, color={0,0,127}));
  connect(const.y, gain.u) annotation (Line(points={{-119,16},{-104,16},{-104,
          -22},{-88,-22}}, color={0,0,127}));
  connect(const.y, mFlow1.m_flow_in) annotation (Line(points={{-119,16},{-8,16},
          {-8,12},{102,12},{102,8}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(extent={{-100,100},{0,-34}}, lineColor={28,108,200}),
        Text(
          extent={{-88,-26},{-88,-26}},
          lineColor={28,108,200},
          textString=""),
        Text(
          extent={{-58,96},{-100,96}},
          lineColor={28,108,200},
          textString="Controller model"),
        Rectangle(extent={{0,100},{100,-34}}, lineColor={28,108,200}),
        Text(
          extent={{42,96},{0,96}},
          lineColor={28,108,200},
          textString="Simulation model")}));
end ShortTermVal;
