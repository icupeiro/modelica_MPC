within BuildingMpc.Fluid.Geothermal.Borefields.Linear;
model Linearisation
  extends Modelica.Icons.Example;
  replaceable package Medium =
      IDEAS.Media.Water "Medium"
      annotation (choicesAllMatching = true);

  Buses.OutputBus                  outputBus annotation (Placement(
        transformation(extent={{-112,58},{-88,82}}),  iconTransformation(extent=
           {{-196,-6},{-176,14}})));
  Buses.InputBus                       inputBus
    annotation (Placement(transformation(extent={{-114,76},{-88,102}})));
  OneUTube oneUTube(
    borFieDat=borFieDat,
    dp_nominal=0,
    redeclare package Medium = Medium,
    Tsoil=273.15 + 13.5,
    TGro_start=(273.15 + 13.5)*ones(10),
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal)
    annotation (Placement(transformation(extent={{-32,-20},{8,20}})));
  IBPSA.Fluid.Sources.Boundary_pT sou(
    redeclare package Medium = Medium,
    use_p_in=false,
    nPorts=1,
    use_T_in=true,
    T=277.15) "Source" annotation (Placement(transformation(extent={{-90,-10},{-70,
            10}}, rotation=0)));
  IBPSA.Fluid.Sources.Boundary_pT sin(
    redeclare package Medium = Medium,
    use_p_in=false,
    use_T_in=false,
    nPorts=1) "Sink" annotation (Placement(transformation(extent={{92,-10},{72,10}},
                  rotation=0)));
  Buildings.Fluid.Movers.FlowControlled_m_flow mFlow(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    allowFlowReversal=false,
    addPowerToMedium=false,
    m_flow_nominal=1,
    use_inputFilter=false)
    annotation (Placement(transformation(extent={{46,-10},{66,10}})));
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorIn(
    redeclare package Medium = Medium,
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
    tau=0,
    T(start=273.15 + 13.5)) "Inlet borehole temperature"
    annotation (Placement(transformation(extent={{-64,-10},{-44,10}})));
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorConOut(
    redeclare package Medium = Medium,
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
    tau=0,
    T(start=273.15 + 12.0))              "Outlet borehole temperature"
    annotation (Placement(transformation(extent={{18,-10},{38,10}})));
  IBPSA.Fluid.Geothermal.Borefields.Data.Borefield.Example
    borFieDat(soiDat=IBPSA.Fluid.Geothermal.Borefields.Data.Soil.SandStone(
        steadyState=false), filDat=
        IBPSA.Fluid.Geothermal.Borefields.Data.Filling.Bentonite(steadyState=false))
    annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
  Modelica.Blocks.Sources.RealExpression realExpression(y=inputBus.Qbor/(mFlow.m_flow
        *Medium.cp_const))
    annotation (Placement(transformation(extent={{12,30},{-8,50}})));
  Modelica.Blocks.Math.Add add(k1=+1)
    annotation (Placement(transformation(extent={{-64,36},{-84,56}})));
  Modelica.Blocks.Sources.RealExpression realExpression1(y=inputBus.Qbor/(
        Medium.cp_const*(TBorIn.T - TBorConOut.T)))
    annotation (Placement(transformation(extent={{-8,78},{12,98}})));
equation
  connect(oneUTube.port_b, TBorConOut.port_a)
    annotation (Line(points={{8,0},{18,0}}, color={0,127,255}));
  connect(sou.ports[1], TBorIn.port_a)
    annotation (Line(points={{-70,0},{-64,0}}, color={0,127,255}));
  connect(oneUTube.port_a, TBorIn.port_b)
    annotation (Line(points={{-32,0},{-44,0}}, color={0,127,255}));
  connect(TBorConOut.port_b, mFlow.port_a)
    annotation (Line(points={{38,0},{46,0}}, color={0,127,255}));
  connect(mFlow.port_b, sin.ports[1])
    annotation (Line(points={{66,0},{72,0}}, color={0,127,255}));
  connect(TBorConOut.T, outputBus.TBorOut) annotation (Line(points={{28,11},{28,
          70.06},{-99.94,70.06}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(add.y, sou.T_in) annotation (Line(points={{-85,46},{-100,46},{-100,4},
          {-92,4}}, color={0,0,127}));
  connect(TBorConOut.T, add.u1)
    annotation (Line(points={{28,11},{28,52},{-62,52}}, color={0,0,127}));
  connect(realExpression.y, add.u2)
    annotation (Line(points={{-9,40},{-62,40}}, color={0,0,127}));
  connect(realExpression1.y, mFlow.m_flow_in)
    annotation (Line(points={{13,88},{56,88},{56,12}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Linearisation;
