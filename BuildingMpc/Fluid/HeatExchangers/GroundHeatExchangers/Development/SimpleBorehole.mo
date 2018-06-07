within BuildingMpc.Fluid.HeatExchangers.GroundHeatExchangers.Development;
model SimpleBorehole "simple borehole model for MPC"
  Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium =
        Medium)
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  IDEAS.Fluid.MixingVolumes.MixingVolume[borFieDat.conDat.nVer] vol(
    m_flow_nominal=m_flow_nominal,
    nPorts=2,
    energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
    allowFlowReversal=false,
    redeclare package Medium = Medium,
    V=borFieDat.conDat.hSeg*Modelica.Constants.pi*borFieDat.conDat.rTub^2)
    annotation (Placement(transformation(extent={{-10,0},{10,20}})));
  parameter Modelica.SIunits.MassFlowRate m_flow_nominal
    "Nominal mass flow rate";
  Modelica.Thermal.HeatTransfer.Components.ThermalResistor[borFieDat.conDat.nVer] Rb(each R=borFieDat.conDat.Rb
        *borFieDat.conDat.rBor) "Borehole resistance"
                          annotation (Placement(transformation(
        extent={{12,-12},{-12,12}},
        rotation=-90,
        origin={-26,34})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature[borFieDat.conDat.nVer]
    prescribedTemperature
    annotation (Placement(transformation(extent={{44,68},{24,88}})));
  Modelica.Blocks.Sources.Constant[borFieDat.conDat.nVer] undisturbedTemp(k=soilTemp)
    "undisturbed ground temperature, could be included as vertical profile"
    annotation (Placement(transformation(extent={{84,68},{64,88}})));

  parameter Modelica.SIunits.Temperature soilTemp = 273.15+10.8 "Constant output value";
  BaseClasses.CylindricalGroundLayer[borFieDat.conDat.nVer]
    lay(
    each soiDat=borFieDat.soiDat,
    each r_a=borFieDat.conDat.rBor,
    each r_b=borFieDat.conDat.rExt,
    each TInt_start=borFieDat.conDat.T_start,
    each TExt_start=borFieDat.conDat.T_start,
    each h=borFieDat.conDat.hSeg,
    each nSta=borFieDat.conDat.nHor)          annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-26,64})));
  replaceable package Medium = Modelica.Media.Interfaces.PartialMedium;

parameter IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.BorefieldData.Template borFieDat "Borefield parameters"
    annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
equation
  connect(port_a, vol[1].ports[1])
    annotation (Line(points={{-100,0},{-2,0}}, color={0,127,255}));
  connect(port_b, vol[borFieDat.conDat.nVer].ports[2])
    annotation (Line(points={{100,0},{0,0}}, color={0,127,255}));
    for i in 1:borFieDat.conDat.nVer-1 loop
      connect(vol[i].ports[2], vol[i+1].ports[1]);
    end for;
  connect(vol.heatPort, Rb.port_a)
    annotation (Line(points={{-10,10},{-26,10},{-26,22}}, color={191,0,0}));
  connect(Rb.port_b, lay.port_a)
    annotation (Line(points={{-26,46},{-26,54}}, color={191,0,0}));
  connect(lay.port_b, prescribedTemperature.port)
    annotation (Line(points={{-26,74},{-26,78},{24,78}}, color={191,0,0}));
  connect(prescribedTemperature.T, undisturbedTemp.y)
    annotation (Line(points={{46,78},{63,78}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-70,80},{70,-80}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={95,95,95},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-62,-52},{62,-60}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-62,58},{62,54}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-62,6},{62,0}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{54,92},{46,-88}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-54,-88},{-46,92}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-72,80},{-62,-80}},
          lineColor={0,0,0},
          fillColor={192,192,192},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{62,80},{72,-80}},
          lineColor={0,0,0},
          fillColor={192,192,192},
          fillPattern=FillPattern.Backward),
        Rectangle(
          extent={{-52,-80},{54,-88}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid)}), Diagram(coordinateSystem(
          preserveAspectRatio=false)));
end SimpleBorehole;
