within BuildingMpc.Fluid.Geothermal.Borefields.BaseClasses;
model CylindricalAxialGroundLayer
  extends CylindricalGroundLayer;
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a[nSta] axial_a "Upper heat port at layer i" annotation (Placement(
        transformation(extent={{-10,88},{10,108}}, rotation=0)));
  Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b[nSta] axial_b "Lower heat port at layer i" annotation (Placement(
        transformation(extent={{-10,-110},{10,-90}}, rotation=0)));

  Modelica.Thermal.HeatTransfer.Components.ThermalConductor[nSta] Gaxial_up(G=G_axial)
    "Borehole resistance" annotation (Placement(transformation(
        extent={{12,-12},{-12,12}},
        rotation=270,
        origin={0,68})));
  Modelica.Thermal.HeatTransfer.Components.ThermalConductor[nSta] Gaxial_down(G=G_axial)
    "Borehole resistance" annotation (Placement(transformation(
        extent={{12,-12},{-12,12}},
        rotation=90,
        origin={0,-52})));
protected
  parameter Modelica.SIunits.ThermalConductance G_axial[nSta](each fixed=false)
    "Axial heat conductance between the temperature nodes";

initial equation
  for i in 1:nSta loop
    G_axial[i] = k*Modelica.Constants.pi*((r[i + 1])^2 - (r[i])^2)/(h/2);
  end for;

equation
  connect(Csoil.port, Gaxial_up.port_a)
    annotation (Line(points={{0,20},{0,56}}, color={191,0,0}));
  connect(Gaxial_up.port_b, axial_a) annotation (Line(points={{2.22045e-15,80},{
          0,80},{0,98}}, color={191,0,0}));
  connect(Gaxial_down.port_a, Csoil.port) annotation (Line(points={{8.88178e-16,
          -40},{0,-40},{0,20}}, color={191,0,0}));
  connect(Gaxial_down.port_b, axial_b)
    annotation (Line(points={{0,-64},{0,-100}}, color={191,0,0}));
end CylindricalAxialGroundLayer;
