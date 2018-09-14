within BuildingMpc.Fluid.Geothermal.Borefields.Linear;
model Linearisation
  extends Modelica.Icons.Example;
  replaceable package Medium =
      IDEAS.Media.Water "Medium"
      annotation (choicesAllMatching = true);

  Buses.OutputBus                  outputBus annotation (Placement(
        transformation(extent={{-112,8},{-88,32}}),   iconTransformation(extent=
           {{-196,-6},{-176,14}})));
  Buses.InputBus                       inputBus
    annotation (Placement(transformation(extent={{-114,26},{-88,52}})));
  Development.OneUTube oneUTube(
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
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorIn1(
    redeclare package Medium = Medium,
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
    tau=0)                               "Inlet borehole temperature"
    annotation (Placement(transformation(extent={{-64,-10},{-44,10}})));
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorConOut(
    redeclare package Medium = Medium,
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
    tau=0)                               "Outlet borehole temperature"
    annotation (Placement(transformation(extent={{18,-10},{38,10}})));
  IBPSA.Fluid.Geothermal.Borefields.Data.Borefield.Example
    borFieDat(soiDat=IBPSA.Fluid.Geothermal.Borefields.Data.Soil.SandStone(
        steadyState=false), filDat=
        IBPSA.Fluid.Geothermal.Borefields.Data.Filling.Bentonite(steadyState=false))
    annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
equation
  connect(oneUTube.port_b, TBorConOut.port_a)
    annotation (Line(points={{8,0},{18,0}}, color={0,127,255}));
  connect(sou.ports[1], TBorIn1.port_a)
    annotation (Line(points={{-70,0},{-64,0}}, color={0,127,255}));
  connect(oneUTube.port_a, TBorIn1.port_b)
    annotation (Line(points={{-32,0},{-44,0}}, color={0,127,255}));
  connect(TBorConOut.port_b, mFlow.port_a)
    annotation (Line(points={{38,0},{46,0}}, color={0,127,255}));
  connect(mFlow.port_b, sin.ports[1])
    annotation (Line(points={{66,0},{72,0}}, color={0,127,255}));
  connect(TBorConOut.T, outputBus.TBorOut) annotation (Line(points={{28,11},{28,
          20.06},{-99.94,20.06}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(mFlow.m_flow_in, inputBus.mFlow) annotation (Line(points={{56,12},{56,
          40},{-100,40}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  connect(sou.T_in, inputBus.TBorIn) annotation (Line(points={{-92,4},{-94,4},{-94,
          38},{-96,38}}, color={0,0,127}), Text(
      string="%second",
      index=1,
      extent={{6,3},{6,3}}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end Linearisation;
