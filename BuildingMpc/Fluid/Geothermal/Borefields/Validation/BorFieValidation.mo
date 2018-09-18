within BuildingMpc.Fluid.Geothermal.Borefields.Validation;
model BorFieValidation
  extends Modelica.Icons.Example;
  package Medium = Modelica.Media.Water.ConstantPropertyLiquidWater;

  parameter Integer nSeg(min=1) = 10
    "Number of segments to use in vertical discretization of the boreholes";
  parameter Integer nHor(min=1) = 10
    "Number of cells to use in radial discretization of soil";
  parameter Modelica.SIunits.Temperature T_start = 273.15 + 10.8
    "Initial soil temperature";

  IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.BorefieldData.ExampleBorefieldData
    borFieDat(           soiDat=
      IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.SoilData.SandStone(
      steadyState=false), filDat=
        IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.FillingData.Bentonite(
         steadyState=false),
  conDat=
      IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.ConfigurationData.ExampleConfigurationData(
      nbBor=4, cooBor={{0,0},{0,6},{6,0},{6,6}}))
    annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
  replaceable
  IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.BorefieldOneUTube borFie(
  redeclare package Medium = Medium,
  borFieDat=borFieDat,
  dp_nominal=10,
  m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
  energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
  massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
  T_start=273.15 + 13.5,
  TMedGro=291.15) constrainedby
    IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Boreholes.BoreholeOneUTube(
  redeclare package Medium = Medium,
  borFieDat=borFieDat,
  dp_nominal=10,
  m_flow_nominal=borFieDat.conDat.mBor_flow_nominal,
  energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
  massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
  T_start=273.15 + 13.5);
end BorFieValidation;
