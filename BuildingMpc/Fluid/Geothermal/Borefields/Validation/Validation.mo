within BuildingMpc.Fluid.Geothermal.Borefields.Validation;
model Validation
  extends Modelica.Icons.Example;
  Modelica.Blocks.Tables.CombiTable1Ds combiTable2D(
    tableOnFile=true,
    tableName="data",
    fileName=ModelicaServices.ExternalReferences.loadResource(
        "modelica://INFRAX/Resources/Validation/borefield_validation.txt"),
    columns={2,3,4,5,6,7})
    annotation (Placement(transformation(extent={{-58,60},{-38,80}})));
  IDEAS.Utilities.Time.ModelTime modTim
    annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
  INFRAX.Data.Parameters.BorefieldData.INFRAX_bF iNFRAX_bF
    annotation (Placement(transformation(extent={{-84,-58},{-64,-38}})));
  IDEAS.Fluid.Sources.Boundary_pT sou(
    redeclare package Medium = Medium,
    use_T_in=true,
    nPorts=1)
    annotation (Placement(transformation(extent={{-94,-28},{-74,-8}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TIn(
    redeclare package Medium = Medium,
    tau=0,
    allowFlowReversal=false,
    m_flow_nominal=iNFRAX_bF.conDat.mBorFie_flow_nominal)
    annotation (Placement(transformation(extent={{-64,-28},{-44,-8}})));
  TwoUTube twoUTube(
    borFieDat=iNFRAX_bF,
    m_flow_nominal=iNFRAX_bF.conDat.mBorFie_flow_nominal,
    allowFlowReversal=false,
    redeclare package Medium = Medium,
    TGro_start=(273.15 + 13.5)*ones(10),
    show_T=true,
    Tsoil=286.65,
    dp_nominal=iNFRAX_bF.conDat.dp_nominal)
    annotation (Placement(transformation(extent={{-34,-38},{6,2}})));
  IDEAS.Fluid.Sensors.TemperatureTwoPort TOut(
    allowFlowReversal=false,
    m_flow_nominal=iNFRAX_bF.conDat.mBorFie_flow_nominal,
    redeclare package Medium = Medium,
    tau=0)
    annotation (Placement(transformation(extent={{16,-28},{36,-8}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource mFlow(
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04,
    redeclare final package Medium = Medium,
    control_dp=false) "Pressure source"
    annotation (Placement(transformation(extent={{46,-28},{66,-8}})));
public
  IDEAS.Fluid.Sources.Boundary_pT sink(redeclare package Medium = Medium,
      nPorts=1)
    annotation (Placement(transformation(extent={{96,-28},{76,-8}})));
  Modelica.Blocks.Math.Gain gain(k=10/3600*Medium.d_const/1000)
    annotation (Placement(transformation(extent={{-24,16},{-4,36}})));
  Modelica.Blocks.Sources.Constant cToK(k=273.15)
    annotation (Placement(transformation(extent={{-26,48},{-14,60}})));
  Modelica.Blocks.Math.Add add
    annotation (Placement(transformation(extent={{-6,62},{14,82}})));
  IBPSA.Fluid.Geothermal.Borefields.TwoUTubes borFie(
    borFieDat=iNFRAX_bF,
    m_flow_nominal=iNFRAX_bF.conDat.mBorFie_flow_nominal,
    allowFlowReversal=false,
    redeclare package Medium = Medium,
    TGro_start=(273.15 + 13.5)*ones(10),
    show_T=true,
    dp_nominal=iNFRAX_bF.conDat.dp_nominal)
    annotation (Placement(transformation(extent={{-34,-90},{6,-50}})));
protected
  IDEAS.Fluid.Movers.BaseClasses.IdealSource mFlow1(
    final allowFlowReversal=false,
    final control_m_flow=true,
    final m_flow_small=1e-04,
    redeclare final package Medium = Medium,
    control_dp=false) "Pressure source"
    annotation (Placement(transformation(extent={{54,-78},{74,-58}})));
equation
  connect(combiTable2D.y[2],add. u1) annotation (Line(points={{-37,70},{-32,70},
          {-32,78},{-8,78}},
                        color={0,0,127}));
  connect(sou.ports[1], TIn.port_a)
    annotation (Line(points={{-74,-18},{-64,-18}}, color={0,127,255}));
  connect(TIn.port_b, twoUTube.port_a)
    annotation (Line(points={{-44,-18},{-34,-18}}, color={0,127,255}));
  connect(twoUTube.port_b, TOut.port_a)
    annotation (Line(points={{6,-18},{16,-18}}, color={0,127,255}));
  connect(TOut.port_b, mFlow.port_a)
    annotation (Line(points={{36,-18},{46,-18}}, color={0,127,255}));
  connect(mFlow.port_b, sink.ports[1])
    annotation (Line(points={{66,-18},{76,-18}}, color={0,127,255}));
  connect(gain.y, mFlow.m_flow_in)
    annotation (Line(points={{-3,26},{50,26},{50,-10}}, color={0,0,127}));
  connect(cToK.y, add.u2)
    annotation (Line(points={{-13.4,54},{-8,54},{-8,66}}, color={0,0,127}));
  connect(add.y, sou.T_in) annotation (Line(points={{15,72},{22,72},{22,70},{28,
          70},{28,38},{-102,38},{-102,-14},{-96,-14}}, color={0,0,127}));
  connect(combiTable2D.y[3], gain.u) annotation (Line(points={{-37,70},{-32,70},
          {-32,26},{-26,26}}, color={0,0,127}));
  connect(modTim.y, combiTable2D.u)
    annotation (Line(points={{-79,70},{-60,70}}, color={0,0,127}));
end Validation;
