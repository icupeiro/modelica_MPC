within BuildingMpc.Examples.SimulationModels.Case900Paper;
model Tuning
  extends Modelica.Icons.Example;
  package Glycol = IBPSA.Media.Antifreeze.PropyleneGlycolWater(property_T=268.15, X_a=0.25);

  parameter Real COP = 5;

  IBPSA.Fluid.Sources.Boundary_pT bou(          redeclare package Medium =
        IDEAS.Media.Air, nPorts=1)
    annotation (Placement(transformation(extent={{-70,30},{-50,50}})));
  inner IDEAS.Buildings.Validation.BaseClasses.SimInfoManagerBestest
                                          sim
    annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
  Modelica.Blocks.Sources.RealExpression optVar2(y=0)
    annotation (Placement(transformation(extent={{40,-6},{20,14}})));
  IDEAS.Fluid.HeatPumps.ScrollWaterToWater heaPum(
    redeclare package Medium1 = IDEAS.Media.Water,
    redeclare package Medium2 = Glycol,
    m1_flow_nominal=0.05,
    m2_flow_nominal=0.05,
    dp1_nominal=10000,
    dp2_nominal=10000,
    redeclare package ref = IDEAS.Media.Refrigerants.R410A,
    enable_temperature_protection=true,
    scaling_factor=(rectangularZoneTemplate.Q_design - rectangularZoneTemplate.QRH_design)
        *0.3/70000,
    datHeaPum=
        IDEAS.Fluid.HeatPumps.Data.ScrollWaterToWater.Heating.DYNACIAT_200_LG_LGP_cissimmo_wetter())
                                                  annotation (Placement(
        transformation(
        extent={{10,-10},{-10,10}},
        rotation=180,
        origin={62,-30})));
public
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow airSystem
    annotation (Placement(transformation(extent={{14,-6},{-6,14}})));
  IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
    redeclare package Medium = IDEAS.Media.Water,
    redeclare
      IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.RadiantSlabChar
      RadSlaCha,
    allowFlowReversal=false,
    A_floor=rectangularZoneTemplate.A,
    dp_nominal=0,
    m_flow_nominal=0.05,
    T_start(displayUnit="K") = 300)
    annotation (Placement(transformation(extent={{-28,-46},{-8,-26}})));
  IBPSA.Fluid.Sources.Boundary_pT sink(
    redeclare package Medium = IDEAS.Media.Water,
    use_T_in=false,
    p=200000,
    nPorts=1) annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={-84,-20})));
  IBPSA.Fluid.Geothermal.Borefields.Data.Borefield.Example borFieDat(
    filDat=IBPSA.Fluid.Geothermal.Borefields.Data.Filling.Bentonite(),
    soiDat=IBPSA.Fluid.Geothermal.Borefields.Data.Soil.SandStone(
        kSoi=1.4,
        cSoi=980,
        dSoi=1358.4),
    conDat=IBPSA.Fluid.Geothermal.Borefields.Data.Configuration.Example(
        mBor_flow_nominal=0.05,
        hBor=30.9,
        dBor=1,
        nBor=1,
        cooBor={{0,0}},
        xC=0.04))
    annotation (Placement(transformation(extent={{-100,-80},{-80,-60}})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow tabs_pump(
    redeclare package Medium = IDEAS.Media.Water,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    allowFlowReversal=false,
    inputType=IDEAS.Fluid.Types.InputType.Continuous,
    addPowerToMedium=false,
    tau=60,
    use_inputFilter=false,
    m_flow_nominal=0.05)
    annotation (Placement(transformation(extent={{30,-88},{10,-68}})));
  IDEAS.Fluid.Movers.FlowControlled_m_flow borFie_pump(
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    allowFlowReversal=false,
    inputType=IDEAS.Fluid.Types.InputType.Continuous,
    addPowerToMedium=false,
    tau=60,
    use_inputFilter=false,
    redeclare package Medium = Glycol,
    m_flow_nominal=0.05)
    annotation (Placement(transformation(extent={{52,70},{72,90}})));
public
  IBPSA.Fluid.Sources.Boundary_pT source(
    p=200000,
    nPorts=1,
    redeclare package Medium = Glycol)
              annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-36,70})));
  IDEAS.Fluid.Geothermal.Borefields.OneUTube borFie(
                           borFieDat=borFieDat, redeclare package Medium =
        Glycol)
    annotation (Placement(transformation(extent={{0,70},{20,90}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_out(
    redeclare package Medium = IDEAS.Media.Water,
    tau=60,
    m_flow_nominal=0.05)
    annotation (Placement(transformation(extent={{-58,-26},{-38,-46}})));
  IDEAS.Controls.Continuous.LimPID conPID(
    Td=0.5,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=0.5,
    Ti=300) annotation (Placement(transformation(extent={{24,-40},{38,-26}})));
public
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow auxHeaSystem
    annotation (Placement(transformation(extent={{12,22},{-8,42}})));
  IDEAS.Controls.Continuous.LimPID conPID1(
    yMin=0,
    yMax=rectangularZoneTemplate.Q_design - rectangularZoneTemplate.QRH_design,
    controllerType=Modelica.Blocks.Types.SimpleController.PI,
    k=5,
    Ti=30)
    annotation (Placement(transformation(extent={{118,42},{98,22}})));

  Modelica.Blocks.Sources.Constant TSetHea(k=21 + 273.15)
    annotation (Placement(transformation(extent={{150,22},{130,42}})));
  Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
    annotation (Placement(transformation(extent={{58,40},{76,58}})));
  Modelica.Blocks.Sources.Constant mFlow(k=0.05)
    annotation (Placement(transformation(extent={{-100,40},{-80,60}})));
  IDEAS.Buildings.Components.RectangularZoneTemplate rectangularZoneTemplate(
    h=2.7,
    redeclare IDEAS.Buildings.Components.ZoneAirModels.WellMixedAir airModel(
        massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState),
    bouTypA=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypB=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypC=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    bouTypCei=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    hasWinCei=false,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.LightRoof conTypCei,
    bouTypFlo=IDEAS.Buildings.Components.Interfaces.BoundaryType.BoundaryWall,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall conTypA,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall conTypB,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall conTypC,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall conTypD,
    hasWinA=true,
    fracA=0,
    redeclare IDEAS.Buildings.Components.Shading.Interfaces.ShadingProperties
      shaTypA,
    bouTypD=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
    aziA=IDEAS.Types.Azimuth.S,
    mSenFac=0.822,
    l=8,
    w=6,
    n50=0.822*0.5*20,
    redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest glazingA,
    redeclare package Medium = IDEAS.Media.Air,
    hasWinB=true,
    hasWinC=true,
    hasWinD=true,
    A_winA=3,
    A_winB=3,
    A_winC=3,
    A_winD=3,
    redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloorFH
      conTypFlo,
    redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest glazingB,
    redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest glazingC,
    redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest glazingD,
    hasInt=true,
    lInt=10,
    redeclare IDEAS.Examples.PPD12.Data.InteriorWall10 conTypInt,
    hasEmb=true,
    T_start=296.15,
    fracB=0,
    fracC=0,
    fracD=0)
    annotation (Placement(transformation(extent={{-50,-14},{-30,6}})));

  IDEAS.Utilities.Time.ModelTime modTim
    annotation (Placement(transformation(extent={{-140,0},{-120,20}})));
  Modelica.Blocks.Tables.CombiTable1Ds combiTable2D(
    tableOnFile=true,
    tableName="data",
    columns={2},
    smoothness=Modelica.Blocks.Types.Smoothness.ConstantSegments,
    fileName=ModelicaServices.ExternalReferences.loadResource(
        "C:/Users/u0112721/Documents/Dymola/Libraries/modelica_MPC/BuildingMpc/HPTset.txt"))
    annotation (Placement(transformation(extent={{-100,0},{-80,20}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_eva_in(
    redeclare package Medium = IDEAS.Media.Water,
    tau=60,
    m_flow_nominal=0.05) annotation (Placement(transformation(
        extent={{10,10},{-10,-10}},
        rotation=90,
        origin={86,-4})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_eva_out(
    redeclare package Medium = IDEAS.Media.Water,
    tau=60,
    m_flow_nominal=0.05) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=90,
        origin={46,-8})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_in(
    redeclare package Medium = IDEAS.Media.Water,
    tau=60,
    m_flow_nominal=0.05) annotation (Placement(transformation(
        extent={{-10,10},{10,-10}},
        rotation=0,
        origin={8,-36})));
equation
  connect(airSystem.Q_flow, optVar2.y)
    annotation (Line(points={{14,4},{19,4}}, color={0,0,127}));
  connect(tabs_pump.port_a, heaPum.port_b1) annotation (Line(points={{30,-78},{86,
          -78},{86,-36},{72,-36}}, color={0,127,255}));
  connect(borFie.port_b, borFie_pump.port_a)
    annotation (Line(points={{20,80},{52,80}}, color={0,127,255}));
  connect(source.ports[1], borFie.port_a) annotation (Line(points={{-26,70},{-14,
          70},{-14,80},{0,80}}, color={0,127,255}));
  connect(T_con_out.port_b, embeddedPipe.port_a)
    annotation (Line(points={{-38,-36},{-28,-36}}, color={0,127,255}));
  connect(T_con_out.port_a, tabs_pump.port_b) annotation (Line(points={{-58,-36},
          {-68,-36},{-68,-78},{10,-78}}, color={0,127,255}));
  connect(sink.ports[1], tabs_pump.port_b) annotation (Line(points={{-84,-30},{-84,
          -36},{-68,-36},{-68,-78},{10,-78}}, color={0,127,255}));
  connect(conPID.y, heaPum.y)
    annotation (Line(points={{38.7,-33},{50,-33}}, color={0,0,127}));
  connect(T_con_out.T, conPID.u_m) annotation (Line(points={{-48,-47},{-48,-54},
          {31,-54},{31,-41.4}}, color={0,0,127}));
  connect(TSetHea.y, conPID1.u_s)
    annotation (Line(points={{129,32},{120,32}}, color={0,0,127}));
  connect(conPID1.y, auxHeaSystem.Q_flow)
    annotation (Line(points={{97,32},{12,32}}, color={0,0,127}));
  connect(temperatureSensor.T, conPID1.u_m)
    annotation (Line(points={{76,49},{108,49},{108,44}}, color={0,0,127}));
  connect(mFlow.y, borFie_pump.m_flow_in) annotation (Line(points={{-79,50},{
          -70,50},{-70,96},{62,96},{62,92}}, color={0,0,127}));
  connect(mFlow.y, tabs_pump.m_flow_in) annotation (Line(points={{-79,50},{-70,
          50},{-70,-14},{-60,-14},{-60,-60},{20,-60},{20,-66}}, color={0,0,127}));
  connect(bou.ports[1], rectangularZoneTemplate.port_a)
    annotation (Line(points={{-50,40},{-38,40},{-38,6}}, color={0,127,255}));
  connect(rectangularZoneTemplate.gainCon, temperatureSensor.port) annotation (
      Line(points={{-30,-7},{-20,-7},{-20,49},{58,49}}, color={191,0,0}));
  connect(auxHeaSystem.port, rectangularZoneTemplate.gainCon) annotation (Line(
        points={{-8,32},{-20,32},{-20,-7},{-30,-7}}, color={191,0,0}));
  connect(airSystem.port, rectangularZoneTemplate.gainCon) annotation (Line(
        points={{-6,4},{-18,4},{-18,-7},{-30,-7}}, color={191,0,0}));
  for i in 1:embeddedPipe.nDiscr loop
  connect(embeddedPipe.heatPortEmb[i], rectangularZoneTemplate.gainEmb[1])    annotation (Line(points={{-18,-26},{-18,-13},{-30,-13}}, color={191,0,0}));
  end for;
  connect(modTim.y,combiTable2D. u) annotation (Line(points={{-119,10},{-102,10}},
                       color={0,0,127}));
  connect(combiTable2D.y[1], conPID.u_s) annotation (Line(points={{-79,10},{-28,
          10},{-28,-33},{22.6,-33}}, color={0,0,127}));
  connect(borFie_pump.port_b, T_eva_in.port_a)
    annotation (Line(points={{72,80},{86,80},{86,6}}, color={0,127,255}));
  connect(T_eva_in.port_b, heaPum.port_a2)
    annotation (Line(points={{86,-14},{86,-24},{72,-24}}, color={0,127,255}));
  connect(heaPum.port_b2, T_eva_out.port_a)
    annotation (Line(points={{52,-24},{46,-24},{46,-18}}, color={0,127,255}));
  connect(T_eva_out.port_b, borFie.port_a) annotation (Line(points={{46,2},{46,
          52},{-14,52},{-14,80},{0,80}}, color={0,127,255}));
  connect(embeddedPipe.port_b, T_con_in.port_a)
    annotation (Line(points={{-8,-36},{-2,-36}}, color={0,127,255}));
  connect(T_con_in.port_b, heaPum.port_a1)
    annotation (Line(points={{18,-36},{52,-36}}, color={0,127,255}));
 annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
            {160,100}})),                                        Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{160,100}})));
end Tuning;
