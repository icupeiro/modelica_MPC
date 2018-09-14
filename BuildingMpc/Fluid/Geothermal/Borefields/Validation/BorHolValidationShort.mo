within BuildingMpc.Fluid.Geothermal.Borefields.Validation;
model BorHolValidationShort
  extends Modelica.Icons.Example;
  package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater;

  parameter Integer nSeg(min=1) = 10
    "Number of segments to use in vertical discretization of the boreholes";
  parameter Integer nHor(min=1) = 10
    "Number of cells to use in radial discretization of soil";
  parameter Modelica.SIunits.Temperature T_start = 273.15 + 13.5
    "Initial soil temperature";

  IBPSA.Fluid.Geothermal.Borefields.Data.Borefield.Example
    borFieDat(           soiDat=
      IBPSA.Fluid.Geothermal.Borefields.Data.Soil.SandStone(
      steadyState=false), filDat=
        IBPSA.Fluid.Geothermal.Borefields.Data.Filling.Bentonite(
         steadyState=false))
    annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
  Modelica.Blocks.Sources.Constant TGroUn(k=273.15 + 13.5)
    "Undisturbed ground temperature" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-80,90})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature preTem
    "Prescribed temperature" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={-50,70})));
  Modelica.Thermal.HeatTransfer.Components.ThermalCollector therCol(m=nSeg)
    "Thermal collector" annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-20,70})));
  IBPSA.Fluid.Geothermal.Borefields.BaseClasses.HeatTransfer.Cylindrical
    lay[nSeg](
    each soiDat=borFieDat.soiDat,
    each r_a=borFieDat.conDat.rBor,
    each r_b=3,
    each h=borFieDat.conDat.hBor/nSeg,
    each nSta=nHor,
    each TInt_start=T_start,
    each TExt_start=T_start)                  annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={0,40})));
  replaceable
    IBPSA.Fluid.Geothermal.Borefields.BaseClasses.Boreholes.OneUTube
    borHolDis(
    redeclare package Medium = Medium,
    borFieDat=borFieDat,
    dp_nominal=10,
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
  energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    each TGro_start=(273.15 + 13.5)*ones(10))
  "Borehole connected to a discrete ground model"                  annotation (
      Placement(transformation(
        extent={{-14,-14},{14,14}},
        rotation=0,
        origin={0,0})));
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorIn(
      redeclare package Medium = Medium, m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
  tau=0)                                 "Inlet borehole temperature"
    annotation (Placement(transformation(extent={{-50,-10},{-30,10}})));
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorDisOut(
      redeclare package Medium = Medium, m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
  tau=0)                                 "Outlet borehole temperature"
    annotation (Placement(transformation(extent={{30,-10},{50,10}})));
  IBPSA.Fluid.Sources.Boundary_pT sin(
    redeclare package Medium = Medium,
    use_p_in=false,
    use_T_in=false,
    nPorts=2,
    p=101330) "Sink" annotation (Placement(transformation(extent={{90,-12},{70,
            8}},  rotation=0)));
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorConOut(
      redeclare package Medium = Medium, m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
  tau=0)                                 "Outlet borehole temperature"
    annotation (Placement(transformation(extent={{30,-70},{50,-50}})));
  IBPSA.Fluid.Sensors.TemperatureTwoPort TBorIn1(
  redeclare package Medium = Medium,
  m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
  tau=0)                                 "Inlet borehole temperature"
    annotation (Placement(transformation(extent={{-50,-70},{-30,-50}})));
Modelica.Blocks.Math.Add add(k2=-1)
  annotation (Placement(transformation(extent={{60,38},{80,58}})));
Modelica.Blocks.Interfaces.RealOutput error
  annotation (Placement(transformation(extent={{100,38},{120,58}})));
  Development.OneUTube oneUTube(
    Tsoil=273.15 + 13.5,
    redeclare package Medium = Medium,
    borFieDat=borFieDat,
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
    dp_nominal=0,
    TGro_start=(273.15 + 13.5)*ones(10),
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
    annotation (Placement(transformation(extent={{-10,-70},{10,-50}})));
  IBPSA.Fluid.Sources.Boundary_pT sour(
    redeclare package Medium = Medium,
    use_p_in=false,
    nPorts=2,
    use_T_in=true,
    p=101330,
    T=277.15) "Source"
                     annotation (Placement(transformation(extent={{-96,-36},{
            -76,-16}},
                  rotation=0)));
  Buildings.Fluid.Movers.FlowControlled_m_flow pum1(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    allowFlowReversal=false,
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
    addPowerToMedium=false)
    annotation (Placement(transformation(extent={{-74,-10},{-54,10}})));
  Buildings.Fluid.Movers.FlowControlled_m_flow pum2(
    redeclare package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    allowFlowReversal=false,
    m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
    addPowerToMedium=false)
    annotation (Placement(transformation(extent={{-74,-70},{-54,-50}})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.Sine sin1(
    amplitude=5,
    freqHz=1/900,
    offset=273.15 + 6.5)
    annotation (Placement(transformation(extent={{-100,30},{-80,50}})));
  Buildings.Controls.OBC.CDL.Continuous.Sources.Pulse pul(
    width=0.7,
    period=2700,
    offset=0,
    amplitude=5)
    annotation (Placement(transformation(extent={{-54,28},{-34,48}})));
equation
  connect(TGroUn.y, preTem.T)
    annotation (Line(points={{-80,79},{-80,70},{-62,70}}, color={0,0,127}));
  connect(preTem.port, therCol.port_b)
    annotation (Line(points={{-40,70},{-30,70}}, color={191,0,0}));
  connect(therCol.port_a, lay.port_b)
    annotation (Line(points={{-10,70},{0,70},{0,50}}, color={191,0,0}));
  connect(lay.port_a, borHolDis.port_wall)
    annotation (Line(points={{0,30},{0,14}}, color={191,0,0}));
  connect(TBorIn.port_b, borHolDis.port_a)
    annotation (Line(points={{-30,0},{-14,0}}, color={0,127,255}));
  connect(borHolDis.port_b, TBorDisOut.port_a)
    annotation (Line(points={{14,0},{30,0}}, color={0,127,255}));
  connect(TBorDisOut.port_b, sin.ports[1])
    annotation (Line(points={{50,0},{70,0},{70,0}}, color={0,127,255}));
  connect(TBorConOut.port_b, sin.ports[2])
    annotation (Line(points={{50,-60},{50,-4},{70,-4}}, color={0,127,255}));
connect(add.y, error)
  annotation (Line(points={{81,48},{110,48}}, color={0,0,127}));
connect(TBorDisOut.T, add.u1) annotation (Line(points={{40,11},{40,54},
        {58,54}}, color={0,0,127}));
connect(TBorConOut.T, add.u2) annotation (Line(points={{40,-49},{48,
        -49},{48,42},{58,42}}, color={0,0,127}));
  connect(TBorIn1.port_b, oneUTube.port_a)
    annotation (Line(points={{-30,-60},{-10,-60}}, color={0,127,255}));
  connect(oneUTube.port_b, TBorConOut.port_a)
    annotation (Line(points={{10,-60},{30,-60}}, color={0,127,255}));
  connect(pum2.port_b, TBorIn1.port_a)
    annotation (Line(points={{-54,-60},{-50,-60}}, color={0,127,255}));
  connect(pum2.port_a, sour.ports[1]) annotation (Line(points={{-74,-60},{-76,
          -60},{-76,-24}}, color={0,127,255}));
  connect(sour.ports[2], pum1.port_a) annotation (Line(points={{-76,-28},{-76,
          -14},{-76,0},{-74,0}}, color={0,127,255}));
  connect(sin1.y, sour.T_in) annotation (Line(points={{-79,40},{-72,40},{-72,20},
          {-98,20},{-98,-22}}, color={0,0,127}));
  connect(pul.y, pum1.m_flow_in) annotation (Line(points={{-33,38},{-22,38},{
          -22,18},{-64,18},{-64,12}}, color={0,0,127}));
  connect(pul.y, pum2.m_flow_in) annotation (Line(points={{-33,38},{-22,38},{
          -22,-40},{-64,-40},{-64,-48}}, color={0,0,127}));
  connect(pum1.port_b, TBorIn.port_a)
    annotation (Line(points={{-54,0},{-50,0}}, color={0,127,255}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)),
  experiment(
    StopTime=86400,
    Tolerance=1e-06,
    __Dymola_fixedstepsize=10,
    __Dymola_Algorithm="Euler"),
  __Dymola_experimentSetupOutput,
  __Dymola_experimentFlags(
    Advanced(
      GenerateVariableDependencies=false,
      OutputModelicaCode=false,
      InlineMethod=0,
      InlineOrder=2,
      InlineFixedStep=0.001),
    Evaluate=false,
    OutputCPUtime=false,
    OutputFlatModelica=false));
end BorHolValidationShort;
