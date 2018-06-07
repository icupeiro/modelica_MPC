within ;
package BuildingMpc "Library of models for building MPC applications"





  package Examples
  extends Modelica.Icons.ExamplesPackage;
    package SimulationModels
    extends Modelica.Icons.ExamplesPackage;
      model Case900TABS "Case900 simulation model"
        extends IDEAS.Buildings.Validation.Cases.Case900(
          redeclare BuildingMpc.Examples.SimulationModels.Envelope.Bui900TABS
            building,
          redeclare HeatingSystems.Hea900TABS heatingSystem,
          redeclare IDEAS.Buildings.Validation.BaseClasses.Occupant.None occupant);
        Modelica.Blocks.Sources.RealExpression[30] states(y=
      {(building.win[1].heaCapGla.T+building.win[2].heaCapGla.T)/2,
      building.wall[3].port_emb[1].T,
      building.wall[3].extCon.port_a.T,
      building.wall[3].layMul.port_gain[2].T,
      building.gF.propsBusInt[1].surfRad.T,
      building.wall[3].layMul.monLay[3].monLayDyn.T[2],
      building.wall[4].port_emb[1].T,
      building.wall[4].extCon.port_a.T,
      building.wall[4].layMul.port_gain[2].T,
      building.gF.propsBusInt[2].surfRad.T,
      building.wall[4].layMul.monLay[3].monLayDyn.T[2],
      building.wall[1].port_emb[1].T,
      building.wall[1].extCon.port_a.T,
      building.wall[1].layMul.port_gain[2].T,
      building.gF.propsBusInt[3].surfRad.T,
      building.wall[1].layMul.monLay[3].monLayDyn.T[2],
      building.wall[2].port_emb[1].T,
      building.wall[2].extCon.port_a.T,
      building.wall[2].layMul.port_gain[2].T,
      building.gF.propsBusInt[4].surfRad.T,
      building.wall[2].layMul.monLay[3].monLayDyn.T[2],
      building.roof.port_emb[1].T,
      building.roof.extCon.port_a.T,
      building.roof.layMul.port_gain[2].T,
      building.gF.propsBusInt[6].surfRad.T,
      building.gF.airModel.vol.dynBal.U,
      building.gF.gainRad.T,
      building.floor.port_emb[1].T,
      building.floor.layMul.port_b.T,
      building.floor.intCon_a.port_a.T})
          annotation (Placement(transformation(extent={{-40,58},{-20,78}})));
      equation
        connect(states.y, heatingSystem.u) annotation (Line(points={{-19,68},{-16,68},
                {-16,10},{-12.2,10}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=31536000,
            __Dymola_NumberOfIntervals=15000,
            Tolerance=1e-06,
            __Dymola_fixedstepsize=20,
            __Dymola_Algorithm="Euler"),
          __Dymola_experimentSetupOutput,
          __Dymola_experimentFlags(
            Advanced(GenerateVariableDependencies=false, OutputModelicaCode=false),
            Evaluate=false,
            OutputCPUtime=false,
            OutputFlatModelica=false));
      end Case900TABS;

      model Case900GEOLoads
        "model to determine the geothermal load to design borefield"
        extends IDEAS.Buildings.Validation.Cases.Case900(redeclare
            BuildingMpc.Examples.SimulationModels.Envelope.Bui900TABSLoad building);
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)));
      end Case900GEOLoads;

      package Envelope
        model Bui900TABS "model to determine the load of the geothermal borefield"
          extends IDEAS.Buildings.Validation.BaseClasses.Structure.Bui900(
          nEmb=1,
          floor(redeclare parameter BuildingMpc.Examples.Data.HeavyFloorTABS
                constructionType))  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        equation
          connect(floor.port_emb, heatPortEmb) annotation (Line(points={{-10,-14.5},{-6,
                  -14.5},{-6,56},{120,56},{120,60},{150,60}}, color={191,0,0}));
        end Bui900TABS;
      end Envelope;

      package HeatingSystems
        model Hea900IdealGEO "ideal geothermal system for case 900"
          extends IDEAS.Templates.Interfaces.BaseClasses.HeatingSystem(
          nZones=1,
          nLoads=1,
          nEmbPorts=1,
          nTemSen=1,
          isCoo=true,
          nConvPorts=1,
          nRadPorts=1,
          P={0},
          Q={0},
          Q_design={2570},
          QHeaSys=max(0,heatPortEmb[1].Q_flow),
          QCooTotal=min(0,heatPortEmb[1].Q_flow));
          IDEAS.Fluid.HeatPumps.Carnot_TCon heaPum(
            redeclare package Medium1 = IDEAS.Media.Water,
            redeclare package Medium2 = IDEAS.Media.Water,
            QCon_flow_nominal=2570,
            dTEva_nominal=-3,
            dTCon_nominal=5,
            use_eta_Carnot_nominal=false,
            COP_nominal=4.5,
            dp1_nominal=0,
            dp2_nominal=0,
            TAppCon_nominal=5,
            TAppEva_nominal=3,
            QCon_flow_max=2570,
            allowFlowReversal1=false,
            allowFlowReversal2=false,
            energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
            m2_flow_nominal=source_pump.m_flow_nominal,
            TCon_nominal=308.15,
            TEva_nominal=276.15)
            annotation (Placement(transformation(extent={{-20,-19},{20,19}},
                rotation=0,
                origin={34,27})));
          IDEAS.Fluid.Sources.Boundary_pT source(
            redeclare package Medium = IDEAS.Media.Water,
            nPorts=2,
            p=200000,
            T=289.15) annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={132,-28})));
          IDEAS.Fluid.Movers.FlowControlled_m_flow source_pump(
            redeclare package Medium = IDEAS.Media.Water,
            massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
            m_flow_nominal=0.16,
            tau=60,
            use_inputFilter=false,
            addPowerToMedium=false,
            allowFlowReversal=false)
            annotation (Placement(transformation(extent={{112,6},{92,26}})));
          IDEAS.Fluid.Sensors.TemperatureTwoPort T_eva_in(
            redeclare package Medium = IDEAS.Media.Water,
            tau=60,
            allowFlowReversal=false,
            m_flow_nominal=source_pump.m_flow_nominal)
            annotation (Placement(transformation(extent={{80,8},{64,24}})));
          IDEAS.Fluid.Sensors.TemperatureTwoPort T_eva_out(
            redeclare package Medium = IDEAS.Media.Water,
            tau=60,
            allowFlowReversal=false,
            m_flow_nominal=source_pump.m_flow_nominal)
            annotation (Placement(transformation(extent={{4,8},{-14,24}})));
          IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
            redeclare package Medium = IDEAS.Media.Water,
            massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
            allowFlowReversal=false,
            energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyStateInitial,
            redeclare Data.TradDesignTABS RadSlaCha,
            m_flow_nominal=0.125,
            A_floor=48)
            annotation (Placement(transformation(extent={{10,-10},{-10,10}},
                rotation=90,
                origin={-178,60})));
          IDEAS.Fluid.Movers.FlowControlled_m_flow sink_pump(
            redeclare package Medium = IDEAS.Media.Water,
            massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
            tau=60,
            use_inputFilter=false,
            addPowerToMedium=false,
            m_flow_nominal=0.125,
            allowFlowReversal=false)
            annotation (Placement(transformation(extent={{-50,28},{-30,48}})));
          IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_in(
            redeclare package Medium = IDEAS.Media.Water,
            tau=60,
            allowFlowReversal=false,
            m_flow_nominal=sink_pump.m_flow_nominal)
            annotation (Placement(transformation(extent={{-20,30},{-4,46}})));
          IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_out(
            redeclare package Medium = IDEAS.Media.Water,
            tau=60,
            allowFlowReversal=false,
            m_flow_nominal=sink_pump.m_flow_nominal)
            annotation (Placement(transformation(extent={{-8,72},{-28,88}})));
          IDEAS.Fluid.HeatExchangers.ConstantEffectiveness passiveCooling(
            redeclare package Medium1 = IDEAS.Media.Water,
            redeclare package Medium2 = IDEAS.Media.Water,
            m2_flow_nominal=source_pump.m_flow_nominal,
            dp1_nominal=0,
            dp2_nominal=0,
            eps=0.9,
            m1_flow_nominal=tabs_pump.m_flow_nominal) annotation (Placement(
                transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={-32,-2})));
          IDEAS.Fluid.MixingVolumes.MixingVolume buffTank(
            redeclare package Medium = IDEAS.Media.Water,
            massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
            m_flow_nominal=sink_pump.m_flow_nominal,
            V=0.1,
            nPorts=4,
            T_start=308.15)
                      annotation (Placement(transformation(extent={{-78,38},{-58,18}})));
          IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor watCon(
            redeclare package Medium = IDEAS.Media.Water,
            energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
            massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
            tau=60,
            m_flow_nominal=sink_pump.m_flow_nominal)
            "3way valve to control water temperature"
            annotation (Placement(transformation(extent={{-78,70},{-98,90}})));
          IDEAS.Fluid.Actuators.Valves.Simplified.ThreeWayValveMotor pasCooCon(
            redeclare package Medium = IDEAS.Media.Water,
            energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
            massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
            tau=60,
            m_flow_nominal=sink_pump.m_flow_nominal)
            "3way valve to control passive cooling"
            annotation (Placement(transformation(extent={{-104,70},{-124,90}})));
          IDEAS.Fluid.Movers.FlowControlled_m_flow tabs_pump(
            redeclare package Medium = IDEAS.Media.Water,
            massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
            tau=60,
            use_inputFilter=false,
            addPowerToMedium=false,
            allowFlowReversal=false,
            m_flow_nominal=0.125)
            annotation (Placement(transformation(extent={{-130,90},{-150,70}})));
          IDEAS.Fluid.Sources.Boundary_pT sink(
            redeclare package Medium = IDEAS.Media.Water,
            nPorts=1,
            p=200000) annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={94,64})));
          IDEAS.Controls.Continuous.LimPID PID_HP_source(
            controllerType=Modelica.Blocks.Types.SimpleController.PI,
            k=1,
            Ti=60,
            yMax=source_pump.m_flow_nominal,
            yMin=source_pump.m_flow_nominal*0.1,
            reverseAction=true)
            annotation (Placement(transformation(extent={{184,36},{168,52}})));
          Modelica.Blocks.Sources.Constant source_set(k=3)
            annotation (Placement(transformation(extent={{214,64},{202,76}})));
          Modelica.Blocks.Sources.RealExpression realExpression(y=T_eva_in.T -
                T_eva_out.T)
            annotation (Placement(transformation(extent={{222,22},{202,42}})));
          IDEAS.Controls.Continuous.LimPID PID_HP_sink(
            controllerType=Modelica.Blocks.Types.SimpleController.PI,
            k=1,
            Ti=60,
            yMax=sink_pump.m_flow_nominal,
            yMin=sink_pump.m_flow_nominal*0.1)
            annotation (Placement(transformation(extent={{112,96},{96,112}})));
          Modelica.Blocks.Sources.RealExpression realExpression1(y=T_con_out.T -
                T_con_in.T)
            annotation (Placement(transformation(extent={{150,82},{130,102}})));
          Modelica.Blocks.Sources.Constant sink_set(k=5)
            annotation (Placement(transformation(extent={{144,122},{130,136}})));
          Modelica.Blocks.Sources.Constant tabs_set(k=5)
            annotation (Placement(transformation(extent={{34,-54},{14,-34}})));
          IDEAS.Fluid.Sensors.TemperatureTwoPort T_tabs_in(
            redeclare package Medium = IDEAS.Media.Water,
            tau=60,
            allowFlowReversal=false,
            m_flow_nominal=tabs_pump.m_flow_nominal)
            annotation (Placement(transformation(extent={{-156,72},{-176,88}})));
          IDEAS.Fluid.Sensors.TemperatureTwoPort T_tabs_out(
            redeclare package Medium = IDEAS.Media.Water,
            tau=60,
            allowFlowReversal=false,
            m_flow_nominal=tabs_pump.m_flow_nominal) annotation (Placement(
                transformation(
                extent={{10,-8},{-10,8}},
                rotation=90,
                origin={-178,38})));
          Modelica.Blocks.Sources.RealExpression heatingMode(y=if runMean.TRm > (273.15
                 + 15.0) then 0.0 else 1.0)
            annotation (Placement(transformation(extent={{-140,104},{-120,124}})));
          IDEAS.Controls.Continuous.LimPID PID_HP_tabs(
            controllerType=Modelica.Blocks.Types.SimpleController.PI,
            k=1,
            Ti=60,
            yMax=tabs_pump.m_flow_nominal,
            yMin=tabs_pump.m_flow_nominal*0.1,
            reverseAction=true)
            annotation (Placement(transformation(extent={{-50,-70},{-66,-54}})));
          Modelica.Blocks.Sources.RealExpression realExpression2(y=if runMean.TRm < (
                273.15 + 12.0) then (T_tabs_in.T - T_tabs_out.T) elseif runMean.TRm > (
                15.0 + 273.15) then (-T_tabs_in.T + T_tabs_out.T) else tabs_set.y)
            annotation (Placement(transformation(extent={{-12,-84},{-32,-64}})));
          IDEAS.Controls.ControlHeating.RunningMeanTemperatureEN15251 runMean(each
              TAveDayIni=10.0*ones(7))
            annotation (Placement(transformation(extent={{-178,-92},{-158,-72}})));
          IDEAS.Controls.Continuous.LimPID heaCurve(
            controllerType=Modelica.Blocks.Types.SimpleController.PI,
            k=1,
            Ti=60,
            yMax=1,
            yMin=0,
            reverseAction=false)
            annotation (Placement(transformation(extent={{-68,108},{-84,124}})));
          Modelica.Blocks.Sources.RealExpression realExpression3(y=T_tabs_in.T)
            annotation (Placement(transformation(extent={{-38,90},{-58,110}})));
          Modelica.Blocks.Tables.CombiTable1D heaCombi(table=[273.15 - 8,273.15 + 37;
                273.15 + 24,273.15 + 24])
            annotation (Placement(transformation(extent={{-18,126},{-38,146}})));
          Modelica.Blocks.Sources.RealExpression realExpression4(y=sim.Te)
            annotation (Placement(transformation(extent={{24,126},{4,146}})));
          Modelica.Blocks.Sources.RealExpression tempControl(y=if runMean.TRm < (273.15
                 + 12.0) then heaCurve.y else 0.0)
            annotation (Placement(transformation(extent={{-138,130},{-118,150}})));
          Modelica.Blocks.Logical.Hysteresis hysteresis(uLow=35, uHigh=40)
            annotation (Placement(transformation(extent={{-50,-38},{-30,-18}})));
          Modelica.Thermal.HeatTransfer.Sensors.TemperatureSensor temperatureSensor
            annotation (Placement(transformation(extent={{-80,-38},{-60,-18}})));
          Modelica.Blocks.Logical.Not not1
            annotation (Placement(transformation(extent={{-22,-36},{-6,-20}})));
          Modelica.Blocks.Sources.RealExpression realExpression5(y=if hysteresis.y
                 then PID_HP_sink.y else 0)
            annotation (Placement(transformation(extent={{58,92},{38,112}})));
          Modelica.Blocks.Sources.Constant sink_set1(k=273.15 + 40)
            annotation (Placement(transformation(extent={{74,122},{60,136}})));
          Modelica.Blocks.Tables.CombiTable1D cooCombi(table=[273.15 + 14,273.15 + 25;
                273.15 + 20,273.15 + 17; 273.15 + 30,273.15 + 15])
            annotation (Placement(transformation(extent={{180,-80},{160,-60}})));
          Modelica.Blocks.Sources.RealExpression realExpression6(y=sim.Te)
            annotation (Placement(transformation(extent={{214,-80},{194,-60}})));
          IDEAS.Controls.Continuous.LimPID cooCurve(
            controllerType=Modelica.Blocks.Types.SimpleController.PI,
            k=1,
            Ti=60,
            yMax=source_pump.m_flow_nominal,
            yMin=0.01*source_pump.m_flow_nominal,
            reverseAction=true)
            annotation (Placement(transformation(extent={{132,-74},{116,-58}})));
          Modelica.Blocks.Sources.RealExpression realExpression7(y=T_tabs_in.T)
            annotation (Placement(transformation(extent={{160,-96},{140,-76}})));
          Modelica.Blocks.Sources.RealExpression realExpression8(y=if runMean.TRm > (
                14.0 + 273.15) then cooCurve.y elseif hysteresis.y then PID_HP_source.y
                 else 0.0)
            annotation (Placement(transformation(extent={{144,30},{124,50}})));
        equation
          connect(source_pump.port_a, source.ports[1])
            annotation (Line(points={{112,16},{130,16},{130,-18}}, color={0,127,255}));
          connect(heaPum.port_a2, T_eva_in.port_b)
            annotation (Line(points={{54,15.6},{54,16},{64,16}}, color={0,127,255}));
          connect(T_eva_in.port_a, source_pump.port_b)
            annotation (Line(points={{80,16},{80,16},{92,16}}, color={0,127,255}));
          connect(T_eva_out.port_a, heaPum.port_b2)
            annotation (Line(points={{4,16},{14,16},{14,15.6}}, color={0,127,255}));
          connect(T_con_in.port_b, heaPum.port_a1)
            annotation (Line(points={{-4,38},{14,38},{14,38.4}}, color={0,127,255}));
          connect(heaPum.port_b1, T_con_out.port_a) annotation (Line(points={{54,38.4},{
                  64,38.4},{64,80},{-8,80}}, color={0,127,255}));
          connect(sink_pump.port_b, T_con_in.port_a)
            annotation (Line(points={{-30,38},{-20,38}}, color={0,127,255}));
          connect(passiveCooling.port_a2, T_eva_out.port_b)
            annotation (Line(points={{-26,8},{-26,16},{-14,16}}, color={0,127,255}));
          connect(passiveCooling.port_b2, source.ports[2]) annotation (Line(points={{-26,
                  -12},{-26,-18},{134,-18}}, color={0,127,255}));
          connect(buffTank.ports[1], sink_pump.port_a)
            annotation (Line(points={{-71,38},{-71,38},{-50,38}}, color={0,127,255}));
          connect(embeddedPipe.heatPortEmb[1], heatPortEmb[1]) annotation (Line(points={{-188,60},
                  {-194,60},{-200,60}},                     color={191,0,0}));
          connect(T_con_out.port_b, buffTank.ports[2]) annotation (Line(points={{-28,80},
                  {-68,80},{-68,38},{-69,38}}, color={0,127,255}));
          connect(watCon.port_b, pasCooCon.port_a1) annotation (Line(points={{-98,80},{-101,
                  80},{-104,80}},      color={0,127,255}));
          connect(pasCooCon.port_a2, passiveCooling.port_b1) annotation (Line(points={{-114,70},
                  {-114,70},{-114,68},{-114,8},{-38,8}},     color={0,127,255}));
          connect(sink.ports[1], T_con_out.port_a)
            annotation (Line(points={{94,74},{94,80},{-8,80}}, color={0,127,255}));
          connect(source_set.y, PID_HP_source.u_s) annotation (Line(points={{201.4,70},
                  {194,70},{194,44},{185.6,44}},color={0,0,127}));
          connect(realExpression.y, PID_HP_source.u_m)
            annotation (Line(points={{201,32},{176,32},{176,34.4}}, color={0,0,127}));
          connect(realExpression1.y, PID_HP_sink.u_m)
            annotation (Line(points={{129,92},{104,92},{104,94.4}},
                                                                 color={0,0,127}));
          connect(sink_set.y, PID_HP_sink.u_s) annotation (Line(points={{129.3,129},{
                  124,129},{124,130},{120,130},{120,104},{113.6,104}},
                                      color={0,0,127}));
          connect(T_tabs_in.port_b, embeddedPipe.port_a) annotation (Line(points={{-176,
                  80},{-178,80},{-178,70}}, color={0,127,255}));
          connect(embeddedPipe.port_b, T_tabs_out.port_a)
            annotation (Line(points={{-178,50},{-178,48}}, color={0,127,255}));
          connect(T_tabs_out.port_b, buffTank.ports[3]) annotation (Line(points={{-178,28},
                  {-118,28},{-108,28},{-88,28},{-88,38},{-67,38}}, color={0,127,255}));
          connect(passiveCooling.port_a1, T_tabs_out.port_b) annotation (Line(points={{-38,
                  -12},{-100,-12},{-178,-12},{-178,28}}, color={0,127,255}));
          connect(watCon.port_a2, T_tabs_out.port_b) annotation (Line(points={{-88,70},
                  {-88,70},{-88,28},{-178,28}},   color={0,127,255}));
          connect(heatingMode.y, pasCooCon.ctrl) annotation (Line(points={{-119,114},{
                  -114,114},{-114,90.8}},
                                    color={0,0,127}));
          connect(realExpression2.y, PID_HP_tabs.u_m) annotation (Line(points={{-33,-74},
                  {-58,-74},{-58,-71.6}}, color={0,0,127}));
          connect(tabs_set.y, PID_HP_tabs.u_s) annotation (Line(points={{13,-44},{-4,-44},
                  {-4,-62},{-48.4,-62}}, color={0,0,127}));
          connect(PID_HP_tabs.y, tabs_pump.m_flow_in) annotation (Line(points={{-66.8,-62},
                  {-140,-62},{-140,68}}, color={0,0,127}));
          connect(watCon.port_a1, buffTank.ports[4]) annotation (Line(points={{-78,80},
                  {-76,80},{-72,80},{-72,60},{-72,38},{-70,38},{-66,38},{-65,38}},
                                      color={0,127,255}));
          connect(pasCooCon.port_b, tabs_pump.port_a)
            annotation (Line(points={{-124,80},{-130,80}}, color={0,127,255}));
          connect(T_tabs_in.port_a, tabs_pump.port_b)
            annotation (Line(points={{-156,80},{-150,80}}, color={0,127,255}));
           heatPortCon.Q_flow = {0};
           heatPortRad.Q_flow = {0};
          connect(realExpression3.y, heaCurve.u_m) annotation (Line(points={{-59,100},{
                  -76,100},{-76,106.4}}, color={0,0,127}));
          connect(heaCombi.y[1], heaCurve.u_s) annotation (Line(points={{-39,136},{-52,
                  136},{-52,116},{-66.4,116}}, color={0,0,127}));
          connect(realExpression4.y, heaCombi.u[1])
            annotation (Line(points={{3,136},{-16,136}}, color={0,0,127}));
          connect(tempControl.y, watCon.ctrl) annotation (Line(points={{-117,140},{-104,
                  140},{-104,90.8},{-88,90.8}}, color={0,0,127}));
          connect(buffTank.heatPort, temperatureSensor.port)
            annotation (Line(points={{-78,28},{-80,28},{-80,-28}}, color={191,0,0}));
          connect(temperatureSensor.T, hysteresis.u)
            annotation (Line(points={{-60,-28},{-52,-28}}, color={0,0,127}));
          connect(hysteresis.y, not1.u)
            annotation (Line(points={{-29,-28},{-23.6,-28}}, color={255,0,255}));
          connect(realExpression5.y, sink_pump.m_flow_in) annotation (Line(points={{37,
                  102},{14,102},{14,50},{-40,50}}, color={0,0,127}));
          connect(sink_set1.y, heaPum.TSet)
            annotation (Line(points={{59.3,129},{10,129},{10,44.1}}, color={0,0,127}));
          connect(realExpression6.y, cooCombi.u[1]) annotation (Line(points={{193,-70},
                  {187.5,-70},{182,-70}}, color={0,0,127}));
          connect(realExpression7.y, cooCurve.u_m) annotation (Line(points={{139,-86},{
                  124,-86},{124,-75.6}}, color={0,0,127}));
          connect(cooCombi.y[1], cooCurve.u_s) annotation (Line(points={{159,-70},{148,
                  -70},{148,-66},{133.6,-66}}, color={0,0,127}));
          connect(realExpression8.y, source_pump.m_flow_in)
            annotation (Line(points={{123,40},{102,40},{102,28}}, color={0,0,127}));
        end Hea900IdealGEO;

        model Hea900TABS
          extends IDEAS.Templates.Interfaces.BaseClasses.HeatingSystem(
          nZones=1,
          nLoads=1,
          nEmbPorts=1,
          nTemSen=1,
          isCoo=true,
          nConvPorts=1,
          nRadPorts=1,
          P={0},
          Q={0},
          Q_design={3000},
          QHeaSys=max(0,heatPortEmb[1].Q_flow),
          QCooTotal=min(0,heatPortEmb[1].Q_flow));
          MPCs.MpcCase900TABS mpcCase900TABS(stateEstimationType=UnitTests.MPC.BaseClasses.StateEstimationType.Perfect)
            annotation (Placement(transformation(extent={{-102,54},{-82,74}})));
          Modelica.Blocks.Sources.RealExpression realExpression(y=mpcCase900TABS.yOpt[1])
            annotation (Placement(transformation(extent={{-130,50},{-150,70}})));
          Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow prescribedHeatFlow
            annotation (Placement(transformation(extent={{-160,50},{-180,70}})));
          Modelica.Blocks.Interfaces.RealInput[mpcCase900TABS.nSta] u annotation (Placement(transformation(
                extent={{-20,-20},{20,20}},
                rotation=-90,
                origin={-122,100})));
        equation
             heatPortCon.Q_flow = {0};
           heatPortRad.Q_flow = {0};
          connect(prescribedHeatFlow.port, heatPortEmb[1])
            annotation (Line(points={{-180,60},{-200,60}}, color={191,0,0}));
          connect(realExpression.y, prescribedHeatFlow.Q_flow)
            annotation (Line(points={{-151,60},{-160,60}}, color={0,0,127}));
          connect(u, mpcCase900TABS.uSta) annotation (Line(points={{-122,100},{-122,83},
                  {-104,83},{-104,66}}, color={0,0,127}));
        end Hea900TABS;
      end HeatingSystems;
    end SimulationModels;

    package ControllerModels
      extends Modelica.Icons.ExamplesPackage;
      model Case900 "Controller model for the BESTEST Case900"
        extends Modelica.Icons.Example;
        IDEAS.Buildings.Components.RectangularZoneTemplate rectangularZoneTemplate(
          h=2.7,
          redeclare package Medium = BuildingMpc.Media.DryAir,
          redeclare IDEAS.Buildings.Components.ZoneAirModels.WellMixedAir
                                                          airModel(massDynamics=
                Modelica.Fluid.Types.Dynamics.SteadyState),
          T_start=293.15,
          bouTypA=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypB=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypC=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypCei=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          hasWinCei=false,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.LightRoof conTypCei,
          bouTypFlo=IDEAS.Buildings.Components.Interfaces.BoundaryType.BoundaryWall,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloor
                                                  conTypFlo,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypA,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypB,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypC,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypD,
          hasWinA=true,
          fracA=0,
          redeclare IDEAS.Buildings.Components.Shading.Interfaces.ShadingProperties
            shaTypA,
          hasWinB=false,
          hasWinC=false,
          hasWinD=false,
          bouTypD=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          aziA=IDEAS.Types.Azimuth.S,
          mSenFac=0.822,
          l=8,
          w=6,
          A_winA=12,
          n50=0.822*0.5*20,
          redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest
            glazingA)
          annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
        inner IDEAS.BoundaryConditions.SimInfoManager sim(lineariseJModelica=true)
          "Simulation information manager for climate data"
          annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
        IBPSA.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
              BuildingMpc.Media.DryAir)
          annotation (Placement(transformation(extent={{-46,30},{-26,50}})));
        Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow preHeaFlo(final
            alpha=0) annotation (Placement(transformation(extent={{40,-22},{20,-2}})));
        Modelica.Blocks.Sources.RealExpression optVar
          annotation (Placement(transformation(extent={{80,-22},{60,-2}})));
      equation
        connect(bou.ports[1], rectangularZoneTemplate.port_a)
          annotation (Line(points={{-26,40},{-8,40},{-8,0}}, color={0,127,255}));
        connect(optVar.y,preHeaFlo. Q_flow)
          annotation (Line(points={{59,-12},{40,-12}}, color={0,0,127}));
        connect(preHeaFlo.port, rectangularZoneTemplate.gainRad) annotation (Line(
              points={{20,-12},{10,-12},{10,-16},{0,-16}}, color={191,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=3.15e+007,
            __Dymola_NumberOfIntervals=15000,
            Tolerance=1e-006,
            __Dymola_fixedstepsize=10,
            __Dymola_Algorithm="Euler"),
          Documentation(info="<html>
<p>
Template for setting up MPC problem using BESTEST case 900 example model. 
Realexpression <code>optVar</code> can be used as a template for
an optimisation variable.
</p>
</html>",       revisions="<html>
<ul>
<li>
November 14, 2016 by Filip Jorissen:<br/>
First implementation
</li>
</ul>
</html>"),__Dymola_Commands(file=
                "Resources/Scripts/Dymola/Buildings/Validation/Tests/ZoneTemplateVerification.mos"
              "Simulate and plot"),
          __Dymola_experimentSetupOutput(events=false),
          __Dymola_experimentFlags(
            Advanced(GenerateVariableDependencies=false, OutputModelicaCode=false),
            Evaluate=false,
            OutputCPUtime=false,
            OutputFlatModelica=false));
      end Case900;

      model Case900TABS
        "Controller model for the BESTEST Case900 with TABS; the optimization variable is the heat flux through TABS"
        extends Modelica.Icons.Example;
        IDEAS.Buildings.Components.RectangularZoneTemplate rectangularZoneTemplate(
          h=2.7,
          redeclare package Medium = BuildingMpc.Media.DryAir,
          redeclare IDEAS.Buildings.Components.ZoneAirModels.WellMixedAir
                                                          airModel(massDynamics=
                Modelica.Fluid.Types.Dynamics.SteadyState),
          bouTypA=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypB=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypC=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypCei=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          hasWinCei=false,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.LightRoof conTypCei,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloor
                                                  conTypFlo,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypA,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypB,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypC,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypD,
          hasWinA=true,
          fracA=0,
          redeclare IDEAS.Buildings.Components.Shading.Interfaces.ShadingProperties
            shaTypA,
          hasWinB=false,
          hasWinC=false,
          hasWinD=false,
          bouTypD=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          aziA=IDEAS.Types.Azimuth.S,
          mSenFac=0.822,
          l=8,
          w=6,
          A_winA=12,
          n50=0.822*0.5*20,
          redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest
            glazingA,
          bouTypFlo=IDEAS.Buildings.Components.Interfaces.BoundaryType.External,
          T_start=293.15)
          annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
        inner IDEAS.BoundaryConditions.SimInfoManager sim(lineariseJModelica=true)
          "Simulation information manager for climate data"
          annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
        IBPSA.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
              BuildingMpc.Media.DryAir)
          annotation (Placement(transformation(extent={{-46,30},{-26,50}})));
        Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow preHeaFlo(final
            alpha=0) annotation (Placement(transformation(extent={{40,-48},{20,-28}})));
        Modelica.Blocks.Sources.RealExpression optVar
          annotation (Placement(transformation(extent={{80,-48},{60,-28}})));
        IDEAS.Buildings.Components.BoundaryWall boundaryWall(
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloor
            constructionType,
          inc=IDEAS.Types.Tilt.Floor,
          azi=rectangularZoneTemplate.aziA,
          A=rectangularZoneTemplate.A) annotation (Placement(transformation(
              extent={{-6,-10},{6,10}},
              rotation=90,
              origin={-8,-38})));
      equation
        connect(bou.ports[1], rectangularZoneTemplate.port_a)
          annotation (Line(points={{-26,40},{-8,40},{-8,0}}, color={0,127,255}));
        connect(optVar.y,preHeaFlo. Q_flow)
          annotation (Line(points={{59,-38},{40,-38}}, color={0,0,127}));
        connect(rectangularZoneTemplate.proBusFlo, boundaryWall.propsBus_a)
          annotation (Line(
            points={{-10,-16},{-10,-33}},
            color={255,204,51},
            thickness=0.5));
        connect(preHeaFlo.port, boundaryWall.port_emb[1])
          annotation (Line(points={{20,-38},{2,-38}}, color={191,0,0}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StartTime=1420000000,
            StopTime=1430000000,
            __Dymola_fixedstepsize=1,
            __Dymola_Algorithm="Euler"),
          Documentation(info="<html>
<p>
Template for setting up MPC problem using BESTEST case 900 example model. 
Realexpression <code>optVar</code> can be used as a template for
an optimisation variable.
</p>
</html>",       revisions="<html>
<ul>
<li>
November 14, 2016 by Filip Jorissen:<br/>
First implementation
</li>
</ul>
</html>"),__Dymola_Commands(file=
                "Resources/Scripts/Dymola/Buildings/Validation/Tests/ZoneTemplateVerification.mos"
              "Simulate and plot"));
      end Case900TABS;

      model Case900TABS_2var
        "Controller model for the BESTEST Case900 with TABS; the optimization variables are the temperature and the flow through the TABS"
        extends Modelica.Icons.Example;
        IDEAS.Buildings.Components.RectangularZoneTemplate rectangularZoneTemplate(
          h=2.7,
          redeclare package Medium = BuildingMpc.Media.DryAir,
          redeclare IDEAS.Buildings.Components.ZoneAirModels.WellMixedAir
                                                          airModel(massDynamics=
                Modelica.Fluid.Types.Dynamics.SteadyState),
          bouTypA=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypB=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypC=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypCei=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          hasWinCei=false,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.LightRoof conTypCei,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloor
                                                  conTypFlo,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypA,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypB,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypC,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypD,
          hasWinA=true,
          fracA=0,
          redeclare IDEAS.Buildings.Components.Shading.Interfaces.ShadingProperties
            shaTypA,
          hasWinB=false,
          hasWinC=false,
          hasWinD=false,
          bouTypD=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          aziA=IDEAS.Types.Azimuth.S,
          mSenFac=0.822,
          l=8,
          w=6,
          A_winA=12,
          n50=0.822*0.5*20,
          redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest
            glazingA,
          T_start=293.15,
          bouTypFlo=IDEAS.Buildings.Components.Interfaces.BoundaryType.External)
          annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
        inner IDEAS.BoundaryConditions.SimInfoManager sim(lineariseJModelica=true)
          "Simulation information manager for climate data"
          annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
        IBPSA.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
              BuildingMpc.Media.DryAir)
          annotation (Placement(transformation(extent={{-46,30},{-26,50}})));
        Modelica.Blocks.Sources.RealExpression optVar1
          annotation (Placement(transformation(extent={{-86,-20},{-66,0}})));
        IDEAS.Buildings.Components.BoundaryWall boundaryWall(
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloor
            constructionType,
          inc=IDEAS.Types.Tilt.Floor,
          azi=rectangularZoneTemplate.aziA,
          A=rectangularZoneTemplate.A) annotation (Placement(transformation(
              extent={{-6,-10},{6,10}},
              rotation=90,
              origin={-8,-38})));
        IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
          redeclare package Medium = IDEAS.Media.Water,
          massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
          redeclare
            IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.RadiantSlabChar
            RadSlaCha,
          m_flow_nominal=5,
          A_floor=rectangularZoneTemplate.A,
          energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
          allowFlowReversal=false)
          annotation (Placement(transformation(extent={{16,-70},{36,-50}})));
        IBPSA.Fluid.Sources.Boundary_pT sink(
          nPorts=2,
          redeclare package Medium = IDEAS.Media.Water,
          use_T_in=true) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-78,-50})));
        Modelica.Blocks.Sources.RealExpression optVar2
          annotation (Placement(transformation(extent={{-86,-34},{-66,-14}})));
    protected
        IDEAS.Fluid.Movers.BaseClasses.IdealSource mflow(
        redeclare final package Medium = IDEAS.Media.Water,
        final control_m_flow=true,
        control_dp=false,
          final allowFlowReversal=false,
          final m_flow_small=1e-04)
                              "Pressure source"
        annotation (Placement(transformation(extent={{-44,-70},{-24,-50}})));
      equation
        connect(bou.ports[1], rectangularZoneTemplate.port_a)
          annotation (Line(points={{-26,40},{-8,40},{-8,0}}, color={0,127,255}));
        connect(rectangularZoneTemplate.proBusFlo, boundaryWall.propsBus_a)
          annotation (Line(
            points={{-10,-16},{-10,-33}},
            color={255,204,51},
            thickness=0.5));
        connect(embeddedPipe.heatPortEmb, boundaryWall.port_emb) annotation (Line(
              points={{26,-50},{26,-50},{26,-38},{2,-38}}, color={191,0,0}));
        connect(embeddedPipe.port_b, sink.ports[1]) annotation (Line(points={{36,-60},
                {54,-60},{54,-82},{-76,-82},{-76,-60}}, color={0,127,255}));
        connect(optVar2.y, sink.T_in) annotation (Line(points={{-65,-24},{-58,-24},{-58,
                -34},{-74,-34},{-74,-38}}, color={0,0,127}));
      connect(mflow.port_b, embeddedPipe.port_a)
        annotation (Line(points={{-24,-60},{16,-60}}, color={0,127,255}));
        connect(mflow.port_a, sink.ports[2])
          annotation (Line(points={{-44,-60},{-80,-60}}, color={0,127,255}));
      connect(optVar1.y, mflow.m_flow_in) annotation (Line(points={{-65,-10},{-40,
              -10},{-40,-52},{-40,-52}}, color={0,0,127}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StartTime=1420000000,
            StopTime=1430000000,
            __Dymola_fixedstepsize=1,
            __Dymola_Algorithm="Euler"),
          Documentation(info="<html>
<p>
Template for setting up MPC problem using BESTEST case 900 example model. 
Realexpression <code>optVar</code> can be used as a template for
an optimisation variable.
</p>
</html>",       revisions="<html>
<ul>
<li>
November 14, 2016 by Filip Jorissen:<br/>
First implementation
</li>
</ul>
</html>"),__Dymola_Commands(file=
                "Resources/Scripts/Dymola/Buildings/Validation/Tests/ZoneTemplateVerification.mos"
              "Simulate and plot"));
      end Case900TABS_2var;

      model Case900TABS_HP
        "Controller model for the BESTEST Case900 with TABS and heat pump with an ideal source; the optimization variables are the outlet temperature of the HP and the mass flows through HP cond/evap"
        extends Modelica.Icons.Example;
        IDEAS.Buildings.Components.RectangularZoneTemplate rectangularZoneTemplate(
          h=2.7,
          redeclare package Medium = BuildingMpc.Media.DryAir,
          redeclare IDEAS.Buildings.Components.ZoneAirModels.WellMixedAir
                                                          airModel(massDynamics=
                Modelica.Fluid.Types.Dynamics.SteadyState),
          bouTypA=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypB=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypC=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypCei=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          hasWinCei=false,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.LightRoof conTypCei,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloor
                                                  conTypFlo,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypA,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypB,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypC,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypD,
          hasWinA=true,
          fracA=0,
          redeclare IDEAS.Buildings.Components.Shading.Interfaces.ShadingProperties
            shaTypA,
          hasWinB=false,
          hasWinC=false,
          hasWinD=false,
          bouTypD=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          aziA=IDEAS.Types.Azimuth.S,
          mSenFac=0.822,
          l=8,
          w=6,
          A_winA=12,
          n50=0.822*0.5*20,
          redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest
            glazingA,
          bouTypFlo=IDEAS.Buildings.Components.Interfaces.BoundaryType.External,
          useFluPor=true,
          T_start=293.15)
          annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
        inner IDEAS.BoundaryConditions.SimInfoManager sim(lineariseJModelica=true)
          "Simulation information manager for climate data"
          annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
        IBPSA.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
              BuildingMpc.Media.DryAir)
          annotation (Placement(transformation(extent={{-46,30},{-26,50}})));
        Modelica.Blocks.Sources.RealExpression optVar1
          annotation (Placement(transformation(extent={{-86,-20},{-66,0}})));
        IDEAS.Buildings.Components.BoundaryWall boundaryWall(
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloor
            constructionType,
          inc=IDEAS.Types.Tilt.Floor,
          azi=rectangularZoneTemplate.aziA,
          A=rectangularZoneTemplate.A) annotation (Placement(transformation(
              extent={{-6,-10},{6,10}},
              rotation=90,
              origin={-8,-38})));
        IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
          redeclare package Medium = IDEAS.Media.Water,
          massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
          redeclare
            IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.RadiantSlabChar
            RadSlaCha,
          allowFlowReversal=false,
          A_floor=rectangularZoneTemplate.A,
          energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
          m_flow_nominal=1)
          annotation (Placement(transformation(extent={{16,-70},{36,-50}})));
        IBPSA.Fluid.Sources.Boundary_pT source(
          redeclare package Medium = IDEAS.Media.Water,
          use_T_in=false,
          nPorts=2,
          p=200000,
          T=287.15)      annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-36,70})));
        Modelica.Blocks.Sources.RealExpression optVar2
          annotation (Placement(transformation(extent={{-86,-34},{-66,-14}})));
    protected
        IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_sink(
          redeclare final package Medium = IDEAS.Media.Water,
          final allowFlowReversal=false,
          final control_m_flow=true,
          m_flow_small=1E-4)        "Pressure source"
          annotation (Placement(transformation(extent={{-44,-70},{-24,-50}})));
    protected
        IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_source(
          redeclare final package Medium = IDEAS.Media.Water,
          final allowFlowReversal=false,
          final control_m_flow=true,
          final m_flow_small=1E-4) "Pressure source"
          annotation (Placement(transformation(extent={{40,62},{60,82}})));
    public
        Modelica.Blocks.Sources.RealExpression optVar3
          annotation (Placement(transformation(extent={{-72,80},{-52,100}})));
        IBPSA.Fluid.Sources.Boundary_pT sink(
          redeclare package Medium = IDEAS.Media.Water,
          use_T_in=false,
          nPorts=1,
          p=200000) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-78,-50})));
        Fluid.HeatPumps.HeatPump heatPump(
          redeclare package Medium1 = IDEAS.Media.Water,
          redeclare package Medium2 = IDEAS.Media.Water,
          dp2_nominal=0,
          dp1_nominal=0,
          Q_nom=3000,
          m1_flow_nominal=1,
          m2_flow_nominal=1,
          COP_expr=4.5)
          annotation (Placement(transformation(extent={{60,-44},{80,-64}})));
      equation
        connect(bou.ports[1], rectangularZoneTemplate.port_a)
          annotation (Line(points={{-26,40},{-8,40},{-8,0}}, color={0,127,255}));
        connect(rectangularZoneTemplate.proBusFlo, boundaryWall.propsBus_a)
          annotation (Line(
            points={{-10,-16},{-10,-33}},
            color={255,204,51},
            thickness=0.5));
        connect(embeddedPipe.heatPortEmb, boundaryWall.port_emb) annotation (Line(
              points={{26,-50},{26,-50},{26,-38},{2,-38}}, color={191,0,0}));
        connect(m_flow_sink.port_b, embeddedPipe.port_a)
          annotation (Line(points={{-24,-60},{16,-60}}, color={0,127,255}));
        connect(optVar1.y, m_flow_sink.m_flow_in) annotation (Line(points={{-65,-10},
                {-40,-10},{-40,-52},{-40,-52}}, color={0,0,127}));
        connect(source.ports[1], m_flow_source.port_a)
          annotation (Line(points={{-26,72},{40,72}},color={0,127,255}));
        connect(optVar3.y, m_flow_source.m_flow_in) annotation (Line(points={{-51,90},
                {-22,90},{44,90},{44,80}},        color={0,0,127}));
        connect(optVar2.y, heatPump.Tcon_out) annotation (Line(points={{-65,-24},{-60,
                -24},{-60,-80},{48,-80},{48,-63},{60,-63}}, color={0,0,127}));
        connect(embeddedPipe.port_b, heatPump.port_a1)
          annotation (Line(points={{36,-60},{60,-60}}, color={0,127,255}));
        connect(sink.ports[1], m_flow_sink.port_a) annotation (Line(points={{-78,-60},
                {-44,-60}},           color={0,127,255}));
        connect(m_flow_source.port_b, heatPump.port_a2) annotation (Line(points={{60,72},
                {60,72},{88,72},{88,-48},{80,-48}}, color={0,127,255}));
        connect(heatPump.port_b2, source.ports[2]) annotation (Line(points={{60,-48},{
                44,-48},{44,52},{-26,52},{-26,68}}, color={0,127,255}));
        connect(m_flow_sink.port_a, heatPump.port_b1) annotation (Line(points={{-44,
                -60},{-44,-90},{80,-90},{80,-60}}, color={0,127,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StartTime=1420000000,
            StopTime=1430000000,
            __Dymola_fixedstepsize=1,
            __Dymola_Algorithm="Euler"),
          Documentation(info="<html>
<p>
Template for setting up MPC problem using BESTEST case 900 example model. 
Realexpression <code>optVar</code> can be used as a template for
an optimisation variable.
</p>
</html>",       revisions="<html>
<ul>
<li>
November 14, 2016 by Filip Jorissen:<br/>
First implementation
</li>
</ul>
</html>"),__Dymola_Commands(file=
                "Resources/Scripts/Dymola/Buildings/Validation/Tests/ZoneTemplateVerification.mos"
              "Simulate and plot"));
      end Case900TABS_HP;

      model Case900GEOTABS_1bor
        "Controller model for the BESTEST Case900 with TABS and heat pump with a single borehole model; the optimization variables are the outlet temperature of the HP and the mass flows through HP cond/evap"
        extends Modelica.Icons.Example;
        IDEAS.Buildings.Components.RectangularZoneTemplate rectangularZoneTemplate(
          h=2.7,
          redeclare package Medium = BuildingMpc.Media.DryAir,
          redeclare IDEAS.Buildings.Components.ZoneAirModels.WellMixedAir
                                                          airModel(massDynamics=
                Modelica.Fluid.Types.Dynamics.SteadyState),
          bouTypA=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypB=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypC=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypCei=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          hasWinCei=false,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.LightRoof conTypCei,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloor
                                                  conTypFlo,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypA,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypB,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypC,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypD,
          hasWinA=true,
          fracA=0,
          redeclare IDEAS.Buildings.Components.Shading.Interfaces.ShadingProperties
            shaTypA,
          hasWinB=false,
          hasWinC=false,
          hasWinD=false,
          bouTypD=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          aziA=IDEAS.Types.Azimuth.S,
          mSenFac=0.822,
          l=8,
          w=6,
          A_winA=12,
          n50=0.822*0.5*20,
          redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest
            glazingA,
          bouTypFlo=IDEAS.Buildings.Components.Interfaces.BoundaryType.External,
          useFluPor=true,
          T_start=293.15)
          annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
        inner IDEAS.BoundaryConditions.SimInfoManager sim(lineariseJModelica=true)
          "Simulation information manager for climate data"
          annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
        IBPSA.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
              BuildingMpc.Media.DryAir)
          annotation (Placement(transformation(extent={{-46,30},{-26,50}})));
        Modelica.Blocks.Sources.RealExpression optVar1
          annotation (Placement(transformation(extent={{-86,-20},{-66,0}})));
        IDEAS.Buildings.Components.BoundaryWall boundaryWall(
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloor
            constructionType,
          inc=IDEAS.Types.Tilt.Floor,
          azi=rectangularZoneTemplate.aziA,
          A=rectangularZoneTemplate.A) annotation (Placement(transformation(
              extent={{-6,-10},{6,10}},
              rotation=90,
              origin={-8,-38})));
        IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
          redeclare package Medium = IDEAS.Media.Water,
          massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
          redeclare
            IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.RadiantSlabChar
            RadSlaCha,
          allowFlowReversal=false,
          A_floor=rectangularZoneTemplate.A,
          energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
          m_flow_nominal=0.2,
          dp_nominal=0)
          annotation (Placement(transformation(extent={{16,-70},{36,-50}})));
        IBPSA.Fluid.Sources.Boundary_pT source(
          redeclare package Medium = IDEAS.Media.Water,
          use_T_in=false,
          p=200000,
          nPorts=1)      annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-36,70})));
        Modelica.Blocks.Sources.RealExpression optVar2
          annotation (Placement(transformation(extent={{-86,-34},{-66,-14}})));
    protected
        IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_sink(
          redeclare final package Medium = IDEAS.Media.Water,
          final allowFlowReversal=false,
          final control_m_flow=true,
          final m_flow_small=1e-04) "Pressure source"
          annotation (Placement(transformation(extent={{-44,-70},{-24,-50}})));
    protected
        IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_source(
          redeclare final package Medium = IDEAS.Media.Water,
          final allowFlowReversal=false,
          final control_m_flow=true,
          final m_flow_small=1e-4) "Pressure source"
          annotation (Placement(transformation(extent={{40,62},{60,82}})));
    public
        Modelica.Blocks.Sources.RealExpression optVar3
          annotation (Placement(transformation(extent={{-72,80},{-52,100}})));
        IBPSA.Fluid.Sources.Boundary_pT sink(
          redeclare package Medium = IDEAS.Media.Water,
          use_T_in=false,
          nPorts=1,
          p=200000) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-78,-50})));
        Fluid.HeatPumps.HeatPump heatPump(
          redeclare package Medium1 = IDEAS.Media.Water,
          redeclare package Medium2 = IDEAS.Media.Water,
          dp2_nominal=0,
          dp1_nominal=0,
          m1_flow_nominal=0.2,
          m2_flow_nominal=0.2)
          annotation (Placement(transformation(extent={{60,-44},{80,-64}})));
        IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.BorefieldData.ExampleBorefieldData
          borFieDat(filDat=
              IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.FillingData.Bentonite(
              steadyState=true), soiDat=
              IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.SoilData.SandStone(
              steadyState=true))
          annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
        Fluid.HeatExchangers.GroundHeatExchangers.SingleBorehole singleBorehole(
          redeclare package Medium = IDEAS.Media.Water,
          soilTemp=273.15 + 10.8,
          borFieDat=borFieDat,
          m_flow_nominal=0.2,
          dp_nominal=0)
          annotation (Placement(transformation(extent={{52,0},{32,20}})));
      equation
        connect(bou.ports[1], rectangularZoneTemplate.port_a)
          annotation (Line(points={{-26,40},{-8,40},{-8,0}}, color={0,127,255}));
        connect(rectangularZoneTemplate.proBusFlo, boundaryWall.propsBus_a)
          annotation (Line(
            points={{-10,-16},{-10,-33}},
            color={255,204,51},
            thickness=0.5));
        connect(embeddedPipe.heatPortEmb, boundaryWall.port_emb) annotation (Line(
              points={{26,-50},{26,-50},{26,-38},{2,-38}}, color={191,0,0}));
        connect(m_flow_sink.port_b, embeddedPipe.port_a)
          annotation (Line(points={{-24,-60},{16,-60}}, color={0,127,255}));
        connect(optVar1.y, m_flow_sink.m_flow_in) annotation (Line(points={{-65,-10},
                {-40,-10},{-40,-52},{-40,-52}}, color={0,0,127}));
        connect(optVar3.y, m_flow_source.m_flow_in) annotation (Line(points={{-51,90},
                {-22,90},{44,90},{44,80}},        color={0,0,127}));
        connect(optVar2.y, heatPump.Tcon_out) annotation (Line(points={{-65,-24},{-60,
                -24},{-60,-80},{48,-80},{48,-63},{60,-63}}, color={0,0,127}));
        connect(embeddedPipe.port_b, heatPump.port_a1)
          annotation (Line(points={{36,-60},{60,-60}}, color={0,127,255}));
        connect(m_flow_sink.port_a, heatPump.port_b1) annotation (Line(points={{-44,-60},
                {-70,-60},{-70,-90},{92,-90},{92,-60},{80,-60}}, color={0,127,255}));
        connect(sink.ports[1], m_flow_sink.port_a) annotation (Line(points={{-78,-60},
                {-61,-60},{-44,-60}}, color={0,127,255}));
        connect(m_flow_source.port_b, heatPump.port_a2) annotation (Line(points={{60,72},
                {60,72},{88,72},{88,-48},{80,-48}}, color={0,127,255}));
        connect(source.ports[1], m_flow_source.port_a) annotation (Line(points={{-26,70},
                {-26,72},{-26,72},{40,72}}, color={0,127,255}));
        connect(heatPump.port_b2,singleBorehole. port_a) annotation (Line(points={{60,
                -48},{60,-48},{60,10},{52,10}}, color={0,127,255}));
        connect(singleBorehole.port_b, m_flow_source.port_a) annotation (Line(points={
                {32,10},{20,10},{20,72},{40,72}}, color={0,127,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=31536000,
            __Dymola_NumberOfIntervals=15000,
            Tolerance=1e-06,
            __Dymola_fixedstepsize=10,
            __Dymola_Algorithm="Euler"),
          Documentation(info="<html>
<p>
Template for setting up MPC problem using BESTEST case 900 example model. 
Realexpression <code>optVar</code> can be used as a template for
an optimisation variable.
</p>
</html>",       revisions="<html>
<ul>
<li>
November 14, 2016 by Filip Jorissen:<br/>
First implementation
</li>
</ul>
</html>"),__Dymola_Commands(file=
                "Resources/Scripts/Dymola/Buildings/Validation/Tests/ZoneTemplateVerification.mos"
              "Simulate and plot"),
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
      end Case900GEOTABS_1bor;

      model Case900GEOTABS
        "Controller model for the BESTEST Case900 with TABS and heat pump with a borefieldl; the optimization variables are the outlet temperature of the HP and the mass flows through HP cond/evap"
        extends Modelica.Icons.Example;
        IDEAS.Buildings.Components.RectangularZoneTemplate rectangularZoneTemplate(
          h=2.7,
          redeclare package Medium = BuildingMpc.Media.DryAir,
          redeclare IDEAS.Buildings.Components.ZoneAirModels.WellMixedAir
                                                          airModel(massDynamics=
                Modelica.Fluid.Types.Dynamics.SteadyState),
          bouTypA=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypB=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypC=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          bouTypCei=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          hasWinCei=false,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.LightRoof conTypCei,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloor
                                                  conTypFlo,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypA,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypB,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypC,
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyWall
                                                 conTypD,
          hasWinA=true,
          fracA=0,
          redeclare IDEAS.Buildings.Components.Shading.Interfaces.ShadingProperties
            shaTypA,
          hasWinB=false,
          hasWinC=false,
          hasWinD=false,
          bouTypD=IDEAS.Buildings.Components.Interfaces.BoundaryType.OuterWall,
          aziA=IDEAS.Types.Azimuth.S,
          mSenFac=0.822,
          l=8,
          w=6,
          A_winA=12,
          n50=0.822*0.5*20,
          redeclare IDEAS.Buildings.Validation.Data.Glazing.GlaBesTest
            glazingA,
          bouTypFlo=IDEAS.Buildings.Components.Interfaces.BoundaryType.External,
          useFluPor=true,
          T_start=293.15)
          annotation (Placement(transformation(extent={{-20,-20},{0,0}})));
        inner IDEAS.BoundaryConditions.SimInfoManager sim(lineariseJModelica=true)
          "Simulation information manager for climate data"
          annotation (Placement(transformation(extent={{-100,60},{-80,80}})));
        IBPSA.Fluid.Sources.Boundary_pT bou(nPorts=1, redeclare package Medium =
              BuildingMpc.Media.DryAir)
          annotation (Placement(transformation(extent={{-46,30},{-26,50}})));
        Modelica.Blocks.Sources.RealExpression optVar1
          annotation (Placement(transformation(extent={{-86,-20},{-66,0}})));
        IDEAS.Buildings.Components.BoundaryWall boundaryWall(
          redeclare IDEAS.Buildings.Validation.Data.Constructions.HeavyFloor
            constructionType,
          inc=IDEAS.Types.Tilt.Floor,
          azi=rectangularZoneTemplate.aziA,
          A=rectangularZoneTemplate.A) annotation (Placement(transformation(
              extent={{-6,-10},{6,10}},
              rotation=90,
              origin={-8,-38})));
        IDEAS.Fluid.HeatExchangers.RadiantSlab.EmbeddedPipe embeddedPipe(
          redeclare package Medium = IDEAS.Media.Water,
          massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
          redeclare
            IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.RadiantSlabChar
            RadSlaCha,
          allowFlowReversal=false,
          A_floor=rectangularZoneTemplate.A,
        energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
        m_flow_nominal=0.2,
        dp_nominal=0)
          annotation (Placement(transformation(extent={{16,-70},{36,-50}})));
        IBPSA.Fluid.Sources.Boundary_pT source(
          redeclare package Medium = IDEAS.Media.Water,
          use_T_in=false,
          p=200000,
          nPorts=1)      annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=0,
              origin={-36,70})));
        Modelica.Blocks.Sources.RealExpression optVar2
          annotation (Placement(transformation(extent={{-86,-34},{-66,-14}})));
    protected
        IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_sink(
          redeclare final package Medium = IDEAS.Media.Water,
          final allowFlowReversal=false,
          final control_m_flow=true,
          final m_flow_small=1e-04) "Pressure source"
          annotation (Placement(transformation(extent={{-44,-70},{-24,-50}})));
    protected
        IDEAS.Fluid.Movers.BaseClasses.IdealSource m_flow_source(
          redeclare final package Medium = IDEAS.Media.Water,
          final allowFlowReversal=false,
          final control_m_flow=true,
          final m_flow_small=1e-4) "Pressure source"
          annotation (Placement(transformation(extent={{40,62},{60,82}})));
    public
        Modelica.Blocks.Sources.RealExpression optVar3
          annotation (Placement(transformation(extent={{-72,80},{-52,100}})));
        IBPSA.Fluid.Sources.Boundary_pT sink(
          redeclare package Medium = IDEAS.Media.Water,
          use_T_in=false,
          nPorts=1,
          p=200000) annotation (Placement(transformation(
              extent={{-10,-10},{10,10}},
              rotation=-90,
              origin={-78,-50})));
        Fluid.HeatPumps.HeatPump heatPump(
          redeclare package Medium1 = IDEAS.Media.Water,
          redeclare package Medium2 = IDEAS.Media.Water,
          dp2_nominal=0,
          dp1_nominal=0,
          Q_nom=3000,
        m2_flow_nominal=0.5,
        m1_flow_nominal=0.5)
          annotation (Placement(transformation(extent={{60,-44},{80,-64}})));
        IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.BorefieldData.ExampleBorefieldData
          borFieDat
          annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
        Fluid.HeatExchangers.GroundHeatExchangers.Development.MultipleBorehole multipleBorehole(
        redeclare package Medium = IDEAS.Media.Water,
        borFieDat=borFieDat,
        m_flow_nominal=0.5,
        dp_nominal=0)
        annotation (Placement(transformation(extent={{52,0},{32,20}})));
      equation
        connect(bou.ports[1], rectangularZoneTemplate.port_a)
          annotation (Line(points={{-26,40},{-8,40},{-8,0}}, color={0,127,255}));
        connect(rectangularZoneTemplate.proBusFlo, boundaryWall.propsBus_a)
          annotation (Line(
            points={{-10,-16},{-10,-33}},
            color={255,204,51},
            thickness=0.5));
        connect(embeddedPipe.heatPortEmb, boundaryWall.port_emb) annotation (Line(
              points={{26,-50},{26,-50},{26,-38},{2,-38}}, color={191,0,0}));
        connect(m_flow_sink.port_b, embeddedPipe.port_a)
          annotation (Line(points={{-24,-60},{16,-60}}, color={0,127,255}));
        connect(optVar1.y, m_flow_sink.m_flow_in) annotation (Line(points={{-65,-10},
                {-40,-10},{-40,-52},{-40,-52}}, color={0,0,127}));
        connect(optVar3.y, m_flow_source.m_flow_in) annotation (Line(points={{-51,90},
                {-22,90},{44,90},{44,80}},        color={0,0,127}));
        connect(optVar2.y, heatPump.Tcon_out) annotation (Line(points={{-65,-24},{-60,
                -24},{-60,-80},{48,-80},{48,-63},{60,-63}}, color={0,0,127}));
        connect(embeddedPipe.port_b, heatPump.port_a1)
          annotation (Line(points={{36,-60},{60,-60}}, color={0,127,255}));
        connect(m_flow_sink.port_a, heatPump.port_b1) annotation (Line(points={{-44,-60},
                {-70,-60},{-70,-90},{92,-90},{92,-60},{80,-60}}, color={0,127,255}));
        connect(sink.ports[1], m_flow_sink.port_a) annotation (Line(points={{-78,-60},
                {-61,-60},{-44,-60}}, color={0,127,255}));
        connect(m_flow_source.port_b, heatPump.port_a2) annotation (Line(points={{60,72},
                {60,72},{88,72},{88,-48},{80,-48}}, color={0,127,255}));
        connect(source.ports[1], m_flow_source.port_a) annotation (Line(points={{-26,70},
                {-26,72},{-26,72},{40,72}}, color={0,127,255}));
      connect(heatPump.port_b2, multipleBorehole.port_a) annotation (Line(
            points={{60,-48},{60,-48},{60,10},{52,10}}, color={0,127,255}));
      connect(multipleBorehole.port_b, m_flow_source.port_a) annotation (Line(
            points={{32,10},{20,10},{20,72},{40,72}}, color={0,127,255}));
        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StartTime=1.42e+009,
            StopTime=1.43e+009,
            __Dymola_fixedstepsize=1,
            __Dymola_Algorithm="Euler"),
          Documentation(info="<html>
<p>
Template for setting up MPC problem using BESTEST case 900 example model. 
Realexpression <code>optVar</code> can be used as a template for
an optimisation variable.
</p>
</html>",       revisions="<html>
<ul>
<li>
November 14, 2016 by Filip Jorissen:<br/>
First implementation
</li>
</ul>
</html>"),__Dymola_Commands(file=
                "Resources/Scripts/Dymola/Buildings/Validation/Tests/ZoneTemplateVerification.mos"
              "Simulate and plot"),
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
      end Case900GEOTABS;
    end ControllerModels;

    package Optimizations
    extends Modelica.Icons.ExamplesPackage;
      model Case900
        extends ControllerModels.Case900(optVar(y=mpcCase900_1.yOpt[1]));
        MPCs.MpcCase900 mpcCase900_1
          annotation (Placement(transformation(extent={{-92,-20},{-72,0}})));
        annotation (
          experiment(
            StopTime=31536000,
            __Dymola_NumberOfIntervals=15000,
            Tolerance=1e-06,
            __Dymola_fixedstepsize=10,
            __Dymola_Algorithm="Euler"),
          __Dymola_experimentSetupOutput,
          __Dymola_experimentFlags(Advanced(
              InlineMethod=0,
              InlineOrder=2,
              InlineFixedStep=0.001)));
      end Case900;

      model Case900TABS
        extends ControllerModels.Case900TABS(
                                          optVar(y=mpcCase900TABS.yOpt[1]));
        MPCs.MpcCase900TABS mpcCase900TABS
          annotation (Placement(transformation(extent={{-90,-22},{-70,-2}})));
        annotation (experiment(
            StopTime=31536000,
            __Dymola_NumberOfIntervals=15000,
            Tolerance=1e-06,
            __Dymola_fixedstepsize=10,
            __Dymola_Algorithm="Euler"));
      end Case900TABS;

      model Case900TABS_2var
        extends ControllerModels.Case900TABS_2var(
                                               optVar1(y=mpcCase900TABS_2var.yOpt[1]),
            optVar2(y=mpcCase900TABS_2var.yOpt[2]));
        MPCs.MpcCase900TABS_2var mpcCase900TABS_2var
          annotation (Placement(transformation(extent={{-92,10},{-72,30}})));
        annotation (
          experiment(
            StopTime=31536000,
            Tolerance=1e-06,
            __Dymola_fixedstepsize=10,
            __Dymola_Algorithm="Euler"),
          __Dymola_experimentSetupOutput,
          __Dymola_experimentFlags(
            Advanced(GenerateVariableDependencies=false, OutputModelicaCode=false),
            Evaluate=false,
            OutputCPUtime=false,
            OutputFlatModelica=false));
      end Case900TABS_2var;

      model Case900TABS_HP
        "Controller model for the BESTEST Case900 with TABS and heat pump with an ideal source; the optimization variables are the outlet temperature of the HP and the mass flows through HP cond/evap"
       extends ControllerModels.Case900TABS_HP(
                                            optVar1(y=mpcCase900TABS_HP.yOpt[1]),
                                            optVar2(y=mpcCase900TABS_HP.yOpt[2]),
                                            optVar3(y=mpcCase900TABS_HP.yOpt[3]));
        MPCs.MpcCase900TABS_HP mpcCase900TABS_HP
          annotation (Placement(transformation(extent={{-84,20},{-64,40}})));
        annotation (experiment(
            StopTime=31536000,
            __Dymola_NumberOfIntervals=15000,
            Tolerance=1e-06,
            __Dymola_fixedstepsize=10,
            __Dymola_Algorithm="Euler"));
      end Case900TABS_HP;

      model Case900GEOTABS_1bor
        "Controller model for the BESTEST Case900 with TABS and heat pump with a single borehole model; the optimization variables are the outlet temperature of the HP and the mass flows through HP cond/evap"
        extends BuildingMpc.Examples.ControllerModels.Case900GEOTABS_1bor(
          optVar3(y=mpcCase900GEOTABS_1bor.yOpt[3]),
          optVar1(y=mpcCase900GEOTABS_1bor.yOpt[1]),
          optVar2(y=mpcCase900GEOTABS_1bor.yOpt[2]),
          singleBorehole(dp_nominal=0));
        MPCs.MpcCase900GEOTABS_1bor mpcCase900GEOTABS_1bor
          annotation (Placement(transformation(extent={{-94,20},{-74,40}})));
                                                                           annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(
          StopTime=31536000,
          __Dymola_NumberOfIntervals=15000,
          Tolerance=1e-06,
          __Dymola_fixedstepsize=10,
          __Dymola_Algorithm="Euler"),
          Documentation(info="<html>
<p>
Template for setting up MPC problem using BESTEST case 900 example model. 
Realexpression <code>optVar</code> can be used as a template for
an optimisation variable.
</p>
</html>",       revisions="<html>
<ul>
<li>
November 14, 2016 by Filip Jorissen:<br/>
First implementation
</li>
</ul>
</html>"),__Dymola_Commands(file=
                "Resources/Scripts/Dymola/Buildings/Validation/Tests/ZoneTemplateVerification.mos"
              "Simulate and plot"),
          __Dymola_experimentSetupOutput(events=false),
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
      end Case900GEOTABS_1bor;

      model Case900GEOTABS
        "Controller model for the BESTEST Case900 with TABS and heat pump with a borefieldl; the optimization variables are the outlet temperature of the HP and the mass flows through HP cond/evap"
          extends BuildingMpc.Examples.ControllerModels.Case900GEOTABS(
          optVar3(y=mpcCase900GEOTABS.yOpt[3]),
          optVar1(y=mpcCase900GEOTABS.yOpt[1]),
          optVar2(y=mpcCase900GEOTABS.yOpt[2]));
        MPCs.MpcCase900GEOTABS_1bor mpcCase900GEOTABS
          annotation (Placement(transformation(extent={{-94,20},{-74,40}})));


        annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
              coordinateSystem(preserveAspectRatio=false)),
          experiment(
            StopTime=31536000,
            __Dymola_NumberOfIntervals=15000,
            Tolerance=1e-06,
            __Dymola_fixedstepsize=10,
            __Dymola_Algorithm="Euler"),
          Documentation(info="<html>
<p>
Template for setting up MPC problem using BESTEST case 900 example model. 
Realexpression <code>optVar</code> can be used as a template for
an optimisation variable.
</p>
</html>",       revisions="<html>
<ul>
<li>
November 14, 2016 by Filip Jorissen:<br/>
First implementation
</li>
</ul>
</html>"),__Dymola_Commands(file=
                "Resources/Scripts/Dymola/Buildings/Validation/Tests/ZoneTemplateVerification.mos"
              "Simulate and plot"),
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
      end Case900GEOTABS;
    end Optimizations;

    package MPCs
    extends Modelica.Icons.ExamplesPackage;

      model MpcCase900
        extends UnitTests.MPC.BaseClasses.Mpc(
          final nOut=3,
          final nOpt=2,
          final nSta=30,
          final nMeas=0,
          final controlTimeStep=3600,
          final nModCorCoeff=16,
          final name= "Case900");
        Modelica.Blocks.Interfaces.RealOutput u = getOutput(tableID,1, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput slack = getOutput(tableID,2, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,3, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        MpcSignalBus bus
          "Bus connector with control variables and outputs"
          annotation (Placement(transformation(extent={{-20,80},{20,120}})));

        expandable connector MpcSignalBus  "Icon for signal bus"
          extends Modelica.Icons.SignalBus;
        Modelica.Blocks.Interfaces.RealInput u;
        Modelica.Blocks.Interfaces.RealInput slack;
        Modelica.Blocks.Interfaces.RealInput Tsta;
        end MpcSignalBus;
      equation
        connect(u, bus.u);
        connect(slack, bus.slack);
        connect(Tsta, bus.Tsta);
      end MpcCase900;

      model MpcCase900TABS
        extends UnitTests.MPC.BaseClasses.Mpc(
          final nOut=3,
          final nOpt=2,
          final nSta=30,
          final nMeas=0,
          final controlTimeStep=3600,
          final nModCorCoeff=21,
          final name= "Case900TABS");
        Modelica.Blocks.Interfaces.RealOutput u = getOutput(tableID,1, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput slack = getOutput(tableID,2, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,3, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        MpcSignalBus bus
          "Bus connector with control variables and outputs"
          annotation (Placement(transformation(extent={{-20,80},{20,120}})));

        expandable connector MpcSignalBus  "Icon for signal bus"
          extends Modelica.Icons.SignalBus;
        Modelica.Blocks.Interfaces.RealInput u;
        Modelica.Blocks.Interfaces.RealInput slack;
        Modelica.Blocks.Interfaces.RealInput Tsta;
        end MpcSignalBus;
      equation
        connect(u, bus.u);
        connect(slack, bus.slack);
        connect(Tsta, bus.Tsta);
      end MpcCase900TABS;

      model MpcCase900TABS_2var
        extends UnitTests.MPC.BaseClasses.Mpc(
          final nOut=4,
          final nOpt=3,
          final nSta=30,
          final nMeas=0,
          final controlTimeStep=3600,
          final nModCorCoeff=21,
          final name= "Case900TABS_2var");
        Modelica.Blocks.Interfaces.RealOutput u1 = getOutput(tableID,1, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput u2 = getOutput(tableID,2, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput slack = getOutput(tableID,3, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,4, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        MpcSignalBus bus
          "Bus connector with control variables and outputs"
          annotation (Placement(transformation(extent={{-20,80},{20,120}})));

        expandable connector MpcSignalBus  "Icon for signal bus"
          extends Modelica.Icons.SignalBus;
        Modelica.Blocks.Interfaces.RealInput u1;
        Modelica.Blocks.Interfaces.RealInput u2;
        Modelica.Blocks.Interfaces.RealInput slack;
        Modelica.Blocks.Interfaces.RealInput Tsta;
        end MpcSignalBus;
      equation
        connect(u1, bus.u1);
        connect(u2, bus.u2);
        connect(slack, bus.slack);
        connect(Tsta, bus.Tsta);
      end MpcCase900TABS_2var;

      model MpcCase900TABS_HP
        extends UnitTests.MPC.BaseClasses.Mpc(
          final nOut=5,
          final nOpt=4,
          final nSta=30,
          final nMeas=0,
          final controlTimeStep=3600,
          final nModCorCoeff=21,
          final name= "Case900TABS_HP");
        Modelica.Blocks.Interfaces.RealOutput u1 = getOutput(tableID,1, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput u2 = getOutput(tableID,2, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput u3 = getOutput(tableID,3, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput slack = getOutput(tableID,4, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,5, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        MpcSignalBus bus
          "Bus connector with control variables and outputs"
          annotation (Placement(transformation(extent={{-20,80},{20,120}})));

        expandable connector MpcSignalBus  "Icon for signal bus"
          extends Modelica.Icons.SignalBus;
        Modelica.Blocks.Interfaces.RealInput u1;
        Modelica.Blocks.Interfaces.RealInput u2;
        Modelica.Blocks.Interfaces.RealInput u3;
        Modelica.Blocks.Interfaces.RealInput slack;
        Modelica.Blocks.Interfaces.RealInput Tsta;
        end MpcSignalBus;
      equation
        connect(u1, bus.u1);
        connect(u2, bus.u2);
        connect(u3, bus.u3);
        connect(slack, bus.slack);
        connect(Tsta, bus.Tsta);
      end MpcCase900TABS_HP;

      model MpcCase900GEOTABS_1bor
        extends UnitTests.MPC.BaseClasses.Mpc(
          final nOut=6,
          final nOpt=4,
          final nSta=140,
          final nMeas=0,
          final controlTimeStep=3600,
          final nModCorCoeff=21,
          final name= "Case900GEOTABS_1bor");
        Modelica.Blocks.Interfaces.RealOutput u1 = getOutput(tableID,1, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput u2 = getOutput(tableID,2, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput u3 = getOutput(tableID,3, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput slack = getOutput(tableID,4, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,5, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput W_comp = getOutput(tableID,6, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        MpcSignalBus bus
          "Bus connector with control variables and outputs"
          annotation (Placement(transformation(extent={{-20,80},{20,120}})));

        expandable connector MpcSignalBus  "Icon for signal bus"
          extends Modelica.Icons.SignalBus;
        Modelica.Blocks.Interfaces.RealInput u1;
        Modelica.Blocks.Interfaces.RealInput u2;
        Modelica.Blocks.Interfaces.RealInput u3;
        Modelica.Blocks.Interfaces.RealInput slack;
        Modelica.Blocks.Interfaces.RealInput Tsta;
        Modelica.Blocks.Interfaces.RealInput W_comp;
        end MpcSignalBus;
      equation
        connect(u1, bus.u1);
        connect(u2, bus.u2);
        connect(u3, bus.u3);
        connect(slack, bus.slack);
        connect(Tsta, bus.Tsta);
        connect(W_comp, bus.W_comp);
      end MpcCase900GEOTABS_1bor;

      model MpcCase900GEOTABS
        extends UnitTests.MPC.BaseClasses.Mpc(
          final nOut=4,
          final nOpt=3,
          final nSta=251,
          final nMeas=0,
          final controlTimeStep=3600,
          final nModCorCoeff=21,
          final name= "Case900GEOTABS");
        Modelica.Blocks.Interfaces.RealOutput u1 = getOutput(tableID,1, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput u2 = getOutput(tableID,2, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput u3 = getOutput(tableID,3, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        Modelica.Blocks.Interfaces.RealOutput Tsta = getOutput(tableID,4, time)
          annotation (Placement(transformation(extent={{96,50},{116,70}})));
        MpcSignalBus bus
          "Bus connector with control variables and outputs"
          annotation (Placement(transformation(extent={{-20,80},{20,120}})));

        expandable connector MpcSignalBus  "Icon for signal bus"
          extends Modelica.Icons.SignalBus;
        Modelica.Blocks.Interfaces.RealInput u1;
        Modelica.Blocks.Interfaces.RealInput u2;
        Modelica.Blocks.Interfaces.RealInput u3;
        Modelica.Blocks.Interfaces.RealInput Tsta;
        end MpcSignalBus;
      equation
        connect(u1, bus.u1);
        connect(u2, bus.u2);
        connect(u3, bus.u3);
        connect(Tsta, bus.Tsta);
      end MpcCase900GEOTABS;
    end MPCs;

    package DesignOpt "design optimizations"
    extends Modelica.Icons.ExamplesPackage;
      model Case900TABS_HP
        "Controller model for the BESTEST Case900 with TABS and heat pump with an ideal source; the optimization variables are the outlet temperature of the HP and the mass flows through HP cond/evap"
        extends BuildingMpc.Examples.ControllerModels.Case900TABS_HP(
                                                                  heatPump(Q_nom=
                desVar.Power_HP), rectangularZoneTemplate(A_winA=desVar.fra_w_window));
        replaceable DecisionVariablesRecords.Case900TABS_HPDesignVariables desVar
          annotation (Placement(transformation(extent={{-86,16},{-66,36}})));
      end Case900TABS_HP;

      package DecisionVariablesRecords
        partial record Case900TABS_HPDesignVariables
          extends Modelica.Icons.Record;
          parameter Real fra_w_window(min=0,max=20) = w_window.val "Fraction of window width";
          parameter Real Power_HP(min=1000, max=5000) = P_HP.val "Max. Power of the HP";
          constant Integer depreciationYears = 25;
          parameter Templates.ContinuousDecisionVariable w_window(
            y_min=8,
            y_max=16,
            investment = w_window.val*200000,
            maintenance=0,
            lifetime = 30,
            replacement= 0,
            interest=0,
            depreciationYears=depreciationYears);
          parameter Templates.ContinuousDecisionVariable P_HP(
            y_min=1000,
            y_max=5000,
            investment = P_HP.val*850,
            maintenance=0.03*P_HP.val,
            lifetime = 20,
            replacement= 2,
            interest=0.02,
            depreciationYears=depreciationYears);
          parameter Real cost = P_HP.cost + w_window.cost;
          annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end Case900TABS_HPDesignVariables;

        package Templates
          record BooleanVariable "Boolean decision variable"
            extends DiscreteDecisionVariable(
              final numOpts=2,
              i=y);
            parameter Integer y;
            final parameter Boolean val = y==2 "Decision variable value";

            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end BooleanVariable;

          record ContinuousDecisionVariable "Continuous decision variable"
            parameter Real y "Normalised design variable";
            parameter Real y_min=0 "Lower range of design variable";
            parameter Real y_max=1 "Upper range of design variable";
            final parameter Real val=y_min+(y_max-y_min)*y "Design variable";
            parameter Real investment "Associated variable annual investment cost difference";
            parameter Real maintenance "Associated variable annual maintenance cost difference";
            parameter Real lifetime "Associated component lifetime in years before replacement occurs";
            parameter Real replacement "Replacement cost after 'lifetime' has expired";
            parameter Real interest "Annual depreciation/interest rate";
            parameter Real depreciationYears "";
            parameter Real cost = costReplacement + costInvestmentMaintenance;
            parameter Real costReplacement=
              investment*replacement*sum({(if mod(integer(j),integer(lifetime))==0 then 1/(1+interest)^j else 0) for j in 1:depreciationYears});
            parameter Real costInvestmentMaintenance=
              investment*(1+maintenance*sum({1/(1+interest)^j for j in 0:depreciationYears-1})) "Maintenance and investment cost using NPV";
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end ContinuousDecisionVariable;

          partial record DiscreteDecisionVariable "Decision variable"
            parameter Real[numOpts] investment "Associated variable initial investment cost";
            parameter Real[numOpts] maintenance "Associated variable annual maintenance cost as fraction of investment cost";
            parameter Integer[numOpts] lifetime "Associated variable lifetime in years before replacement occurs";
            parameter Real[numOpts] replacement "Replacement cost after 'lifetime' has expired as fraction of investment cost";
            parameter Real[numOpts] interest "Annual depreciation/interest rate";
            parameter Integer depreciationYears = 25 "Duration for NPV calculations";
            parameter Integer numOpts(start=1,min=1) "Number of optimisation variables";
            parameter Integer i(start=1, min=1, max=numOpts) "Decision variable index";
            parameter Real cost = costReplacement + costInvestmentMaintenance;
            parameter Real depreciation = sum({1/(1+interest[i])^j for j in 0:depreciationYears-1});
            parameter Real costReplacement=investment[i]*replacement[i]*floor((depreciationYears-0.001)/lifetime[i]);

            parameter Real costInvestmentMaintenance=investment[i]*(1 + maintenance[i]*depreciation) "Maintenance and investment cost using NPV";
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end DiscreteDecisionVariable;

          record IntegerVariable "Boolean decision variable"
            extends Templates.DiscreteDecisionVariable(
              final i=y);
            final parameter Integer val = vals[y] "Design variable value";
            parameter Integer y(start=1) "Actual decision variable from stochastic algorithm";
            parameter Integer[:] vals "Design variable values";
            annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end IntegerVariable;
        end Templates;
      end DecisionVariablesRecords;
    end DesignOpt;

    package Data
      record TradDesignTABS
        "traditional design of TABS for BESTEST example following standards EN1264, 11855 and UPONOR guidelines"
        extends IDEAS.Fluid.HeatExchangers.RadiantSlab.BaseClasses.RadiantSlabChar(
        tabs=false,
        T=0.11,
        d_a= 0.016,
        s_r = 0.002,
        S_1=0.03,
        S_2=0.08-S_1,
        c_b = 1000,
        rho_b=1400);  //Rb is 0.05 (m2K)/W
      end TradDesignTABS;

      record HeavyFloorTABS "BESTEST heavy floor with TABS"
        extends IDEAS.Buildings.Data.Interfaces.Construction(
          incLastLay = IDEAS.Types.Tilt.Floor,
          locGain = {2},
          final mats={
            IDEAS.Buildings.Validation.Data.Insulation.InsulationFloor(d=1.007),
            IDEAS.Buildings.Validation.Data.Materials.ConcreteSlab(d=0.08-0.0565),
            IDEAS.Buildings.Validation.Data.Materials.ConcreteSlab(d=0.0565)});
      end HeavyFloorTABS;
    end Data;
  end Examples;

  package Fluid
  extends Modelica.Icons.Package;

    package HeatExchangers
    extends Modelica.Icons.VariantsPackage;
      package GroundHeatExchangers
      extends Modelica.Icons.VariantsPackage;
        model SingleBorehole "single borehole model for MPC"

          replaceable package Medium =
              Modelica.Media.Interfaces.PartialMedium "Medium through the borehole"
              annotation (choicesAllMatching = true);

          BaseClasses.SingleBoreHoleUTube
            borehole(
            allowFlowReversal=false,
            computeFlowResistance=false,
            massDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
            redeclare package Medium = Medium,
            m_flow_nominal=m_flow_nominal,
            dp_nominal=dp_nominal,
            intHex(
            Q2_flow(       nominal = -65*borFieDat.conDat.hSeg),
            Q1_flow( nominal = 65*borFieDat.conDat.hSeg)),
            borFieDat=borFieDat,
          energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState)
                                                annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=0,
                origin={0,0})));
          BaseClasses.CylindricalGroundLayer
            lay[borFieDat.conDat.nVer](
            each h=borFieDat.conDat.hSeg,
            each r_a=borFieDat.conDat.rBor,
            each r_b=3,
            each nSta=borFieDat.conDat.nHor,
            each TInt_start=borFieDat.conDat.T_start,
            each TExt_start=borFieDat.conDat.T_start,
            each soiDat=borFieDat.soiDat)                   annotation (Placement(
                transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={0,40})));
          Modelica.Thermal.HeatTransfer.Sources.PrescribedTemperature[borFieDat.conDat.nVer]
            prescribedTemperature
            annotation (Placement(transformation(extent={{40,50},{20,70}})));
          Modelica.Blocks.Sources.Constant[borFieDat.conDat.nVer] tempProfile(k=
                soilTemp)
            "undisturbed ground temperature, could be included as vertical profile"
            annotation (Placement(transformation(extent={{80,50},{60,70}})));
          parameter Modelica.SIunits.Temperature soilTemp=273.15 + 10.8
            "Undisturbed temperature of the ground";
          Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium =
                Medium)
            annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
          Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium =
                Medium)
            annotation (Placement(transformation(extent={{90,-10},{110,10}})));
          parameter Modelica.SIunits.MassFlowRate m_flow_nominal
            "Nominal mass flow rate through the borehole"
            annotation (Dialog(group="Nominal condition"));
          parameter Modelica.SIunits.PressureDifference dp_nominal
            "Pressure difference through the borehole"
            annotation (Dialog(group="Nominal condition"));
          parameter
            IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.BorefieldData.Template
            borFieDat "Borefield parameters";
      protected
            parameter Modelica.SIunits.SpecificHeatCapacity cpMed=
              Medium.specificHeatCapacityCp(Medium.setState_pTX(
              Medium.p_default,
              Medium.T_default,
              Medium.X_default)) "Specific heat capacity of the fluid";
          parameter Modelica.SIunits.ThermalConductivity kMed=
              Medium.thermalConductivity(Medium.setState_pTX(
              Medium.p_default,
              Medium.T_default,
              Medium.X_default)) "Thermal conductivity of the fluid";
          parameter Modelica.SIunits.DynamicViscosity mueMed=Medium.dynamicViscosity(
              Medium.setState_pTX(
              Medium.p_default,
              Medium.T_default,
              Medium.X_default)) "Dynamic viscosity of the fluid";

        equation
          connect(tempProfile.y, prescribedTemperature.T)
            annotation (Line(points={{59,60},{42,60}}, color={0,0,127}));
          connect(prescribedTemperature.port, lay.port_b)
            annotation (Line(points={{20,60},{0,60},{0,50}}, color={191,0,0}));
          connect(lay.port_a, borehole.port_wall) annotation (Line(points={{-4.44089e-16,
                  30},{0,30},{0,10}}, color={191,0,0}));
          connect(port_a, borehole.port_a)
            annotation (Line(points={{-100,0},{-10,0}},         color={0,127,255}));
          connect(borehole.port_b, port_b)
            annotation (Line(points={{10,0},{100,0}},         color={0,127,255}));
                         annotation (Placement(transformation(extent={{-10,-10},{10,10}})),
                      Icon(coordinateSystem(preserveAspectRatio=false), graphics={
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
                  fillPattern=FillPattern.Solid)}),                      Diagram(
                coordinateSystem(preserveAspectRatio=false)));
        end SingleBorehole;

        package Development
          model MultipleBorehole "multiple borehole model for MPC"

            replaceable package Medium =
                Modelica.Media.Interfaces.PartialMedium "Medium through the borehole"
                annotation (choicesAllMatching = true);

            Modelica.Fluid.Interfaces.FluidPort_a port_a(redeclare package Medium =
                  Medium)
              annotation (Placement(transformation(extent={{-110,-10},{-90,10}})));
            Modelica.Fluid.Interfaces.FluidPort_b port_b(redeclare package Medium =
                  Medium)
              annotation (Placement(transformation(extent={{90,-10},{110,10}})));

            parameter
              IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.BorefieldData.Template
              borFieDat "Borefield parameters";

            SingleBorehole singleBorehole(
              redeclare package Medium = Medium,
              soilTemp=soilTemp,
              borFieDat=borFieDat,
              dp_nominal=dp_nominal,
              m_flow_nominal=m_flow_nominal)
              annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
            IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.BaseClasses.MassFlowRateMultiplier
                                               masFloDiv(
              redeclare package Medium = Medium,
              k=borFieDat.conDat.nbBh,
            allowFlowReversal=false)   "Division of flow rate"
              annotation (Placement(transformation(extent={{-60,-10},{-80,10}})));
            IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.BaseClasses.MassFlowRateMultiplier
                                               masFloMul(
              redeclare package Medium = Medium,
              k=borFieDat.conDat.nbBh,
            allowFlowReversal=false)   "Mass flow multiplier"
              annotation (Placement(transformation(extent={{60,-10},{80,10}})));
            parameter Modelica.SIunits.Temperature soilTemp=273.15 + 10.8
              "Undisturbed temperature of the ground";
            parameter Modelica.SIunits.MassFlowRate m_flow_nominal
              "Nominal mass flow rate through the borehole";
            parameter Modelica.SIunits.PressureDifference dp_nominal
              "Pressure difference through the borehole";
          equation

            assert(borFieDat.conDat.nbBh >= 1, "incorrect amount of boreholes for the borefield");

            connect(port_a, masFloDiv.port_b)
              annotation (Line(points={{-100,0},{-80,0}}, color={0,127,255}));
            connect(masFloDiv.port_a, singleBorehole.port_a) annotation (Line(points={{-60,
                    0},{-36,0},{-36,0},{-10,0}}, color={0,127,255}));
            connect(singleBorehole.port_b, masFloMul.port_a)
              annotation (Line(points={{10,0},{60,0}}, color={0,127,255}));
            connect(masFloMul.port_b, port_b)
              annotation (Line(points={{80,0},{100,0}}, color={0,127,255}));
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
                    fillPattern=FillPattern.Solid)}),                      Diagram(
                  coordinateSystem(preserveAspectRatio=false)));
          end MultipleBorehole;

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
            replaceable package Medium =
              Modelica.Media.Interfaces.PartialMedium;

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
        end Development;

        package BaseClasses
          model CylindricalGroundLayer
            "Heat conduction in a cylinder using the radial discretization as advised by Eskilson"
            parameter Integer nSta(min=4) = 10 "number of discretisations";
            parameter
              IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.SoilData.Template soiDat
              annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
            parameter Modelica.SIunits.Height h "Height of the cylinder";
            parameter Modelica.SIunits.Radius r_a "Internal radius";
            parameter Modelica.SIunits.Radius r_b "External radius";
            parameter Modelica.SIunits.Temperature TInt_start=293.15
              "Initial temperature at port_a, used if steadyStateInitial = false"
              annotation (Dialog(group="Initialization", enable=not steadyStateInitial));
            parameter Modelica.SIunits.Temperature TExt_start=293.15
              "Initial temperature at port_b, used if steadyStateInitial = false"
              annotation (Dialog(group="Initialization", enable=not steadyStateInitial));
            parameter Boolean steadyStateInitial=true
              "true initializes dT(0)/dt=0, false initializes T(0) at fixed temperature using T_a_start and T_b_start"
              annotation (Dialog(group="Initialization"), Evaluate=true);

            parameter Real gridFac(min=1) = 2 "Grid factor for spacing";

            Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a(T(start=TInt_start))
              "Heat port at surface a" annotation (Placement(transformation(extent={{-110,
                      -10},{-90,10}}, rotation=0)));
            Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_b(T(start=TExt_start))
              "Heat port at surface b" annotation (Placement(transformation(extent={{90,-10},
                      {110,10}},rotation=0)));

            parameter Modelica.SIunits.Radius r[nSta + 1](each fixed=false)
              "Radius to the boundary of the i-th domain";
            parameter Real beta[nSta] = 2*ones(nSta);
            Modelica.Thermal.HeatTransfer.Components.ThermalResistor[nSta] Rsoil(R = {1/G[i] for i in 1:nSta})
              "Borehole resistance" annotation (Placement(transformation(
                  extent={{12,-12},{-12,12}},
                  rotation=180,
                  origin={0,0})));
            Modelica.Thermal.HeatTransfer.Components.HeatCapacitor[nSta] Csoil(C=C, der_T(
                  fixed=true))
              annotation (Placement(transformation(extent={{-12,42},{12,66}})));

        protected
            parameter Modelica.SIunits.Radius rC[nSta] = {(r[i] + r[i + 1])/2 for i in 1:nSta}
              "Radius to the center of the i-th domain";

            final parameter Modelica.SIunits.SpecificHeatCapacity c=soiDat.c
              "Specific heat capacity";
            final parameter Modelica.SIunits.ThermalConductivity k=soiDat.k
              "Thermal conductivity of the material";
            final parameter Modelica.SIunits.Density d=soiDat.d
              "Density of the material";

            parameter Modelica.SIunits.ThermalConductance G[nSta] = cat(1, {2*Modelica.Constants.pi*k*h/Modelica.Math.log(rC[1]/r_a)}, {2*Modelica.Constants.pi*k*h/Modelica.Math.log(rC[i]/rC[i - 1]) for i in 2:nSta-1}, {2*Modelica.Constants.pi*k*h/Modelica.Math.log(r_b/rC[nSta])})
              "Heat conductance between the temperature nodes";
            parameter Modelica.SIunits.HeatCapacity C[nSta] = {d*Modelica.Constants.pi*c*h*((r[i+1]^2 - (r[i])^2)) for i in 1:nSta}
                  "Heat capacity of each state";
            parameter Real gridFac_sum(fixed=false);
            parameter Real gridFac_sum_old(fixed=false);

          initial algorithm
            for i in 0:nSta - 3 - 1 loop
              if i == 0 then
                gridFac_sum := gridFac^i;
                gridFac_sum_old := gridFac_sum;
              else
                gridFac_sum := gridFac_sum_old + gridFac^i;
                gridFac_sum_old := gridFac_sum;
              end if;
            end for;
          initial equation
            r[1] = r_a;
            r[2] = r_a + sqrt(k/c/d*60) "eskilson minimum length";
            r[3] = r_a + 2*sqrt(k/c/d*60);
            r[4] = r_a + 3*sqrt(k/c/d*60);

            for i in 5:nSta + 1 loop
              r[i] = r[i - 1] + (r_b - r[4])/gridFac_sum*gridFac^(i - 5);
            end for;

          equation

            connect(port_a, Rsoil[1].port_a)
              annotation (Line(points={{-100,0},{-12,0}}, color={191,0,0}));
            connect(Rsoil[nSta].port_a, Csoil[nSta].port)  annotation (Line(points={{-12,0},
                    {-10,0},{-10,32},{0,32},{0,42}},
                                                color={191,0,0}));
            connect(Rsoil[nSta].port_b, port_b)
              annotation (Line(points={{12,0},{100,0}}, color={191,0,0}));
            connect(Rsoil[nSta].port_b, Csoil[nSta].port);
            for i in 1:(nSta-1) loop
              connect(Rsoil[i].port_a, Csoil[i].port);
              connect(Rsoil[i].port_b, Rsoil[i+1].port_a) annotation (Line(points={{12,0},{28,0},{28,
                    -22},{-24,-22},{-24,0},{-12,0},{-12,1.44329e-15}}, color={191,0,0}));
            end for;






            annotation (
              Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                      100,100}})),
              Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,-100},{100,100}}),
                              graphics={
                  Rectangle(
                    extent={{-94,4},{92,-4}},
                    lineColor={0,0,0},
                    fillColor={191,0,0},
                    fillPattern=FillPattern.Solid),
                  Polygon(
                    points={{12,8},{14,8},{16,4},{18,-2},{18,-6},{16,-12},{10,-16},{6,-20},
                        {-2,-22},{-6,-18},{-12,-12},{-14,-2},{-12,4},{-10,8},{-8,10},{-6,
                        12},{-2,14},{2,14},{8,12},{12,8}},
                    lineColor={0,0,0},
                    smooth=Smooth.None,
                    fillColor={215,215,215},
                    fillPattern=FillPattern.Solid),
                  Polygon(
                    points={{-6,-16},{2,-18},{8,-16},{14,-14},{10,-16},{6,-20},{-2,-22},{
                        -8,-20},{-12,-12},{-14,-2},{-12,4},{-10,8},{-8,10},{-10,0},{-10,-8},
                        {-6,-16}},
                    lineColor={0,0,0},
                    smooth=Smooth.None,
                    fillColor={135,135,135},
                    fillPattern=FillPattern.Solid),
                  Text(
                    extent={{-110,-74},{-26,-86}},
                    lineColor={0,0,255},
                    textString="%r_a"),
                  Text(
                    extent={{-22,-62},{20,-76}},
                    lineColor={0,0,255},
                    textString="%nSta"),
                  Text(
                    extent={{16,-76},{102,-88}},
                    lineColor={0,0,255},
                    textString="%r_b"),
                  Polygon(
                    points={{-50,60},{-38,34},{-32,0},{-36,-30},{-50,-60},{-62,-60},{-48,
                        -30},{-44,0},{-50,34},{-62,60},{-50,60}},
                    lineColor={0,0,0},
                    smooth=Smooth.None,
                    fillPattern=FillPattern.Backward,
                    fillColor={175,175,175}),
                  Polygon(
                    points={{52,60},{64,34},{70,0},{66,-30},{52,-60},{40,-60},{54,-30},{
                        58,0},{52,34},{40,60},{52,60}},
                    lineColor={0,0,0},
                    smooth=Smooth.None,
                    fillPattern=FillPattern.Backward,
                    fillColor={175,175,175}),
                  Text(
                    extent={{-100,100},{100,60}},
                    lineColor={0,0,255},
                    textString="%name")}),
              defaultComponentName="lay",
              Documentation(info="<html>
<p>
Model for radial heat transfer in a hollow cylinder.
</p>
<p>
If the heat capacity of the material is non-zero, then this model computes transient heat conduction, i.e., it
computes a numerical approximation to the solution of the heat equation
</p>
<p align=\"center\" style=\"font-style:italic;\">
   &rho; c ( &part; T(r,t) &frasl; &part;t ) = 
    k ( &part;&sup2; T(r,t) &frasl; &part;r&sup2; + 1 &frasl; r &nbsp;  &part; T(r,t) &frasl; &part;r ),
</p>
<p>
where 
<i>&rho;</i>
is the mass density,
<i>c</i>
is the specific heat capacity per unit mass,
<i>T</i>
is the temperature at location <i>r</i> and time <i>t</i> and
<i>k</i> is the heat conductivity. 
At the locations <i>r=r<sub>a</sub></i> and <i>r=r<sub>b</sub></i>, 
the temperature and heat flow rate are equal to the 
temperature and heat flow rate of the heat ports.
</p>
<p>
If the heat capacity of the material is set to zero, then steady-state heat flow is computed using
</p>
<p align=\"center\" style=\"font-style:italic;\">
   Q = 2 &pi; k (T<sub>a</sub>-T<sub>b</sub>)&frasl; ln(r<sub>a</sub> &frasl; r<sub>b</sub>),
</p>
<p>
where
<i>r<sub>a</sub></i> is the internal radius,
<i>r<sub>b</sub></i> is the external radius,
<i>T<sub>a</sub></i> is the temperature at port a and
<i>T<sub>b</sub></i> is the temperature at port b.
</p>
<h4>Implementation</h4>
<p>
To spatially discretize the heat equation, the construction is 
divided into compartments with <code>material.nSta &ge; 1</code> state variables. 
The state variables are connected to each other through thermal conductors. 
There is also a thermal conductor
between the surfaces and the outermost state variables. Thus, to obtain
the surface temperature, use <code>port_a.T</code> (or <code>port_b.T</code>)
and not the variable <code>T[1]</code>.
</p>
</html>",           revisions="<html>
<ul>
<li>
Januari, 2014, by Damien Picard:<br/>
Modify the discretization of the cilindrical layer so that the first three layers have an equal thickness the following an exponentionally growing thickness.
This follows the guidelines of Eskilson (P. Eskilson. Thermal analysis of heat extraction
boreholes. PhD thesis, Dep. of Mathematical
Physics, University of Lund, Sweden, 1987).
</li>
<li>
March 9, 2012, by Michael Wetter:<br/>
Removed protected variable <code>der_T</code> as it is not required.
</li>
<li>
April 14 2011, by Pierre Vigouroux:<br/>
First implementation.
</li>
</ul>
</html>"));
          end CylindricalGroundLayer;

          model InternalHEXUTube
            "Internal part of a borehole for a U-Tube configuration"

            extends IBPSA.Fluid.Interfaces.FourPortHeatMassExchanger(
              redeclare final package Medium1 = Medium,
              redeclare final package Medium2 = Medium,
              T1_start=T_start,
              T2_start=T_start,
              final tau1=Modelica.Constants.pi*borFieDat.conDat.rTub^2*borFieDat.conDat.hSeg*rho1_nominal/
                  m1_flow_nominal,
              final tau2=Modelica.Constants.pi*borFieDat.conDat.rTub^2*borFieDat.conDat.hSeg*rho2_nominal/
                  m2_flow_nominal,
              final vol1(
                final energyDynamics=energyDynamics,
                final massDynamics=massDynamics,
                final prescribedHeatFlowRate=false,
                final m_flow_small=m1_flow_small,
                final V=borFieDat.conDat.volOneLegSeg,
                final mSenFac=mSenFac),
              redeclare final IBPSA.Fluid.MixingVolumes.MixingVolume vol2(
                final energyDynamics=energyDynamics,
                final massDynamics=massDynamics,
                final prescribedHeatFlowRate=false,
                final m_flow_small=m2_flow_small,
                final V=borFieDat.conDat.volOneLegSeg,
                final mSenFac=mSenFac));
            replaceable package Medium =
                Modelica.Media.Interfaces.PartialMedium "Medium"
                annotation (choicesAllMatching = true);
            parameter Real mSenFac=1
                "Factor for scaling the sensible thermal mass of the volume"
                annotation (Dialog(group="Advanced"));
            parameter Boolean dynFil=true
                "Set to false to remove the dynamics of the filling material"
                annotation (Dialog(tab="Dynamics"));
            parameter Modelica.SIunits.Temperature T_start
              "Initial temperature of the filling material and fluid"
              annotation (Dialog(group="Filling material"));
            parameter
              IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.BorefieldData.Template
              borFieDat "Borefield parameters"
              annotation (Placement(transformation(extent={{-100,-120},{-80,-100}})));

            Modelica.Blocks.Sources.RealExpression RVol1(y=
             BuildingMpc.Fluid.HeatExchangers.GroundHeatExchangers.BaseClasses.convectionResistance(
                      hSeg=borFieDat.conDat.hSeg,
                      rBor=borFieDat.conDat.rBor,
                      rTub=borFieDat.conDat.rTub,
                      eTub=borFieDat.conDat.eTub,
                      kMed=kMed,
                      mueMed=mueMed,
                      cpMed=cpMed,
                      m_flow_nominal=m1_flow_nominal))
              "Convective and thermal resistance at fluid 1"
              annotation (Placement(transformation(extent={{-100,-2},{-80,18}})));
            Modelica.Blocks.Sources.RealExpression RVol2(y=
             BuildingMpc.Fluid.HeatExchangers.GroundHeatExchangers.BaseClasses.convectionResistance(
                      hSeg=borFieDat.conDat.hSeg,
                      rBor=borFieDat.conDat.rBor,
                      rTub=borFieDat.conDat.rTub,
                      eTub=borFieDat.conDat.eTub,
                      kMed=kMed,
                      mueMed=mueMed,
                      cpMed=cpMed,
                      m_flow_nominal=m2_flow_nominal))
              "Convective and thermal resistance at fluid 2"
               annotation (Placement(transformation(extent={{-100,-18},{-80,2}})));

            IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Boreholes.BaseClasses.InternalResistancesOneUTube
              intResUTub(
              dynFil=dynFil,
              T_start=T_start,
              energyDynamics=energyDynamics,
              Rgb_val=Rgb_val,
              Rgg_val=Rgg_val,
              RCondGro_val=RCondGro_val,
              x=x,
              borFieDat=borFieDat)
              "Internal resistances for a single U-tube configuration"
              annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
            Modelica.Thermal.HeatTransfer.Components.ConvectiveResistor RConv2
              "Pipe convective resistance"
              annotation (Placement(transformation(extent={{-12,12},{12,-12}},
                  rotation=270,
                  origin={0,-28})));
            Modelica.Thermal.HeatTransfer.Components.ConvectiveResistor RConv1
              "Pipe convective resistance"
              annotation (Placement(transformation(extent={{-12,-12},{12,12}},
                  rotation=90,
                  origin={0,28})));
            Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_wall
              "Thermal connection for borehole wall"
              annotation (Placement(transformation(extent={{-10,90},{10,110}})));

        protected
            parameter Modelica.SIunits.HeatCapacity Co_fil=borFieDat.filDat.d*borFieDat.filDat.c*borFieDat.conDat.hSeg*Modelica.Constants.pi
                *(borFieDat.conDat.rBor^2 - 2*(borFieDat.conDat.rTub + borFieDat.conDat.eTub)^2)
              "Heat capacity of the whole filling material";

            parameter Modelica.SIunits.SpecificHeatCapacity cpMed=
                Medium.specificHeatCapacityCp(Medium.setState_pTX(
                Medium.p_default,
                Medium.T_default,
                Medium.X_default)) "Specific heat capacity of the fluid";
            parameter Modelica.SIunits.ThermalConductivity kMed=
                Medium.thermalConductivity(Medium.setState_pTX(
                Medium.p_default,
                Medium.T_default,
                Medium.X_default)) "Thermal conductivity of the fluid";
            parameter Modelica.SIunits.DynamicViscosity mueMed=Medium.dynamicViscosity(
                Medium.setState_pTX(
                Medium.p_default,
                Medium.T_default,
                Medium.X_default)) "Dynamic viscosity of the fluid";

           parameter Real Rgb_val = (1 - x)*Rg
              "Thermal resistance between grout zone and borehole wall";
            parameter Real Rgg_val = 2*Rgb_val*(Rar - 2*x*Rg)/(2*Rgb_val - Rar + 2*x*Rg) "Thermal resistance between the two grout zones";
            parameter Real RCondGro_val =  x*Rg + RCondPipe/borFieDat.conDat.hSeg
              "Thermal resistance between: pipe wall to capacity in grout";
            parameter Real x = Modelica.Math.log(sqrt(borFieDat.conDat.rBor^2 + 2*borFieDat.conDat.rTub^2)/(2*borFieDat.conDat.rTub))/
                Modelica.Math.log(borFieDat.conDat.rBor/(sqrt(2)*borFieDat.conDat.rTub)) "Capacity location";

            // ********** Rb and Ra from multipole **********
            // Help variables
            parameter Real sigma= (borFieDat.filDat.k - borFieDat.soiDat.k)/(borFieDat.filDat.k + borFieDat.soiDat.k);

            parameter Real rTub_in = borFieDat.conDat.rTub-borFieDat.conDat.eTub "Inner radius of tube";

            parameter Real RCondPipe(unit="(m.K)/W") =  Modelica.Math.log((borFieDat.conDat.rTub)/rTub_in)/(2*Modelica.Constants.pi*borFieDat.conDat.kTub)
              "Thermal resistance of the pipe wall";
            parameter Real RConv = BuildingMpc.Fluid.HeatExchangers.GroundHeatExchangers.BaseClasses.convectionResistance(
                      hSeg=borFieDat.conDat.hSeg,
                      rBor=borFieDat.conDat.rBor,
                      rTub=borFieDat.conDat.rTub,
                      eTub=borFieDat.conDat.eTub,
                      kMed=kMed,
                      mueMed=mueMed,
                      cpMed=cpMed,
                      m_flow_nominal=(m1_flow_nominal+m2_flow_nominal)/2);

            parameter Real beta= 2*Modelica.Constants.pi*borFieDat.filDat.k*(RCondPipe+RConv);

            parameter Real R_1delta_LS= 1/(2*Modelica.Constants.pi*borFieDat.filDat.k)*(log(borFieDat.conDat.rBor/borFieDat.conDat.rTub) + log(borFieDat.conDat.rBor/(2*borFieDat.conDat.xC)) +
              sigma*log(borFieDat.conDat.rBor^4/(borFieDat.conDat.rBor^4 - borFieDat.conDat.xC^4))) + RCondPipe + RConv;

            parameter Real R_1delta_MP= R_1delta_LS - 1/(2*Modelica.Constants.pi*borFieDat.filDat.k)*(borFieDat.conDat.rTub^2/
              (4*borFieDat.conDat.xC^2)*(1 - sigma*4*borFieDat.conDat.xC^4/(borFieDat.conDat.rBor^4 - borFieDat.conDat.xC^4))^2)/((1 + beta)/(1 - beta) +
              borFieDat.conDat.rTub^2/(4*borFieDat.conDat.xC^2)*(1 + sigma*16*borFieDat.conDat.xC^4*borFieDat.conDat.rBor^4/(borFieDat.conDat.rBor^4 - borFieDat.conDat.xC^4)^2));

            parameter Real Ra_LS =     1/(Modelica.Constants.pi*borFieDat.filDat.k)*(log(2*borFieDat.conDat.xC/borFieDat.conDat.rTub) + sigma*log((
              borFieDat.conDat.rBor^2 + borFieDat.conDat.xC^2)/(borFieDat.conDat.rBor^2 - borFieDat.conDat.xC^2)))  + 2*(RCondPipe + RConv);

            //Rb and Ra
            parameter Real Rb_internal= if borFieDat.conDat.use_Rb then borFieDat.conDat.Rb else R_1delta_MP/2;
            parameter Real Ra= Ra_LS - 1/(Modelica.Constants.pi*borFieDat.filDat.k)*(borFieDat.conDat.rTub^2/(4*borFieDat.conDat.xC^2)*(1 + sigma*
              4*borFieDat.conDat.rBor^4*borFieDat.conDat.xC^2/(borFieDat.conDat.rBor^4 - borFieDat.conDat.xC^4))/((1 + beta)/(1 - beta) - borFieDat.conDat.rTub^2/(4*borFieDat.conDat.xC^2) +
              sigma*2*borFieDat.conDat.rTub^2*borFieDat.conDat.rBor^2*(borFieDat.conDat.rBor^4 + borFieDat.conDat.xC^4)/(borFieDat.conDat.rBor^4 - borFieDat.conDat.xC^4)^2));

            //Conversion of Rb (resp. Ra) to Rg (resp. Rar) of Bauer:
            parameter Real Rg = (2*Rb_internal-RCondPipe-RConv)/borFieDat.conDat.hSeg;
            parameter Real Rar= (Ra-2*(RCondPipe + RConv))/borFieDat.conDat.hSeg;

          equation
              assert(borFieDat.conDat.borHolCon == IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Types.BoreHoleConfiguration.SingleUTube,
            "This model should be used for single U-type borefield, not double U-type. 
  Check that the record General has been correctly parametrized");
            if dynFil then
            end if;

            connect(RVol2.y, RConv2.Rc) annotation (Line(points={{-79,-8},{-60,-8},{-40,
                    -8},{-40,-28},{-12,-28}},
                                          color={0,0,127}));
            connect(RVol1.y, RConv1.Rc) annotation (Line(points={{-79,8},{-40,8},{-40,28},
                    {-12,28}}, color={0,0,127}));
            connect(intResUTub.port_wall, port_wall) annotation (Line(points={{10,0},{26,
                    0},{40,0},{40,100},{0,100}}, color={191,0,0}));
            connect(vol1.heatPort, RConv1.fluid) annotation (Line(points={{-10,60},{-20,
                    60},{-20,40},{6.66134e-016,40}}, color={191,0,0}));
            connect(RConv1.solid, intResUTub.port_1)
              annotation (Line(points={{0,16},{0,16},{0,10}}, color={191,0,0}));
            connect(RConv2.fluid, vol2.heatPort) annotation (Line(points={{0,-40},{20,-40},
                    {20,-60},{12,-60}}, color={191,0,0}));
            connect(RConv2.solid, intResUTub.port_2)
              annotation (Line(points={{0,-16},{0,-16},{0,-10}}, color={191,0,0}));
              annotation (Dialog(tab="Dynamics"),
              Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},{100,
                      100}})),
              Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-120},{100,
                      100}}), graphics={Rectangle(
                    extent={{88,54},{-88,64}},
                    lineColor={0,0,255},
                    pattern=LinePattern.None,
                    fillColor={0,0,255},
                    fillPattern=FillPattern.Solid), Rectangle(
                    extent={{88,-66},{-88,-56}},
                    lineColor={0,0,255},
                    pattern=LinePattern.None,
                    fillColor={0,0,255},
                    fillPattern=FillPattern.Solid)}),
              Documentation(info="<html>
<p>
Model for the heat transfer between the fluid and within the borehole filling. 
This model computes the dynamic response of the fluid in the tubes, 
the heat transfer between the fluid and the borehole filling, 
and the heat storage within the fluid and the borehole filling.
</p>
<p>
This model computes the different thermal resistances present 
in a single-U-tube borehole using the method of Bauer et al. (2011) 
and computing explicitely the fluid-to-ground thermal resistance 
<i>R<sub>b</sub></i> and the 
grout-to-grout resistance
<i>R<sub>a</sub></i> as defined by Hellstroem (1991)
using the multipole method.
The multipole method is implemented in
<a href=\"modelica://IBPSA.Fluid.HeatExchangers.Boreholes.BaseClasses.singleUTubeResistances\">
IBPSA.Fluid.HeatExchangers.Boreholes.BaseClasses.singleUTubeResistances</a>. 
The convection resistance is calculated using the 
Dittus-Boelter correlation
as implemented in
<a href=\"modelica://IBPSA.Fluid.HeatExchangers.Boreholes.BaseClasses.convectionResistance\">
IBPSA.Fluid.HeatExchangers.Boreholes.BaseClasses.convectionResistance</a>. 
</p>
<p>
The figure below shows the thermal network set up by Bauer et al. (2010).
</p>
<p align=\"center\">
<img alt=\"image\" src=\"modelica://IDEAS/Resources/Images/Fluid/HeatExchangers/Boreholes/BaseClasses/Bauer_singleUTube.png\"/>
</p>
<h4>References</h4>
<p>
G. Hellstr&ouml;m. 
<i>Ground heat storage: thermal analyses of duct storage systems (Theory)</i>. 
Dept. of Mathematical Physics, University of Lund, Sweden, 1991.
</p>
<p>
D. Bauer, W. Heidemann, H. M&uuml;ller-Steinhagen, and H.-J. G. Diersch.
<i>
<a href=\"http://dx.doi.org/10.1002/er.1689\">
Thermal resistance and capacity models for borehole heat exchangers
</a>
</i>.
International Journal Of Energy Research, 35:312&ndash;320, 2011.
</p>
</html>",           revisions="<html>
<p>
<ul>
<li>
June 18, 2014, by Michael Wetter:<br/>
Added initialization for temperatures and derivatives of <code>capFil1</code>
and <code>capFil2</code> to avoid a warning during translation.
</li>
<li>
February 14, 2014, by Michael Wetter:<br/>
Removed unused parameters <code>B0</code> and <code>B1</code>.
</li>
<li>
January 24, 2014, by Michael Wetter:<br/>
Revised implementation, added comments, replaced 
<code>HeatTransfer.Windows.BaseClasses.ThermalConductor</code>
with resistance models from the Modelica Standard Library.
</li>
<li>
January 23, 2014, by Damien Picard:<br/>
First implementation.
</li>
</ul>
</p>
</html>"),    Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                      100,100}}), graphics));
          end InternalHEXUTube;

          function convectionResistance
            "Thermal resistance from the fluid in pipes and the grout zones (Bauer et al. 2011)"

            // Geometry of the borehole
            input Modelica.SIunits.Height hSeg "Height of the element";
            input Modelica.SIunits.Radius rBor "Radius of the borehole";
            input Modelica.SIunits.Radius rTub "Tube radius";
            input Modelica.SIunits.Length eTub "Tube thickness";
            // thermal properties
            input Modelica.SIunits.ThermalConductivity kMed
              "Thermal conductivity of the fluid";
            input Modelica.SIunits.DynamicViscosity mueMed
              "Dynamic viscosity of the fluid";
            input Modelica.SIunits.SpecificHeatCapacity cpMed
              "Specific heat capacity of the fluid";
            input Modelica.SIunits.MassFlowRate m_flow_nominal "Nominal mass flow rate";

            // Outputs
            output Modelica.SIunits.ThermalResistance RFlu2pipe
              "Convection resistance (or conduction in fluid if no mass flow)";

        protected
            parameter Modelica.SIunits.Radius rTub_in = rTub - eTub;
            Modelica.SIunits.CoefficientOfHeatTransfer h
              "Convective heat transfer coefficient of the fluid";

            Real k(unit="s/kg")
              "Coefficient used in the computation of the convective heat transfer coefficient";
          algorithm
            // ********** Convection resistance **********
            // Dittus-Boelter: h = 0.023*k_f*Re*Pr/(2*rTub)
            // Re = rho*v*DTub / mue_f = m_flow/(pi r^2) * DTub/mue_f = 2*m_flow / ( mue*pi*rTub)
            // Convection

            RFlu2pipe := 1/(2*Modelica.Constants.pi*rTub_in*hSeg*(0.023*kMed*(cpMed*mueMed/kMed)^(0.35)/(2*rTub_in)*((2/(mueMed*Modelica.Constants.pi*rTub_in))*m_flow_nominal)^0.8));

            annotation (Diagram(graphics), Documentation(info="<html>
<p>
This model computes the convection resistance in the pipes of a borehole segment 
with heigth <i>h<sub>Seg</sub></i>.
</p>
<p>
The correlation of Dittus-Boelter (1930) is used to find the convection heat transfer coefficient as
</p>
<p align=\"center\" style=\"font-style:italic;\">
  Nu = 0.023 &nbsp; Re<sup>0.8</sup> &nbsp; Pr<sup>n</sup>,
</p>
<p>
where <i>Nu</i> is the Nusselt number, 
<i>Re</i> is the Reynolds number and 
<i>Pr</i> is the Prandlt number.
We selected <i>n=0.35</i>, as the reference uses <i>n=0.4</i> for heating and 
<i>n=0.3</i> for cooling.
Dittus-Boelter&apos;s correlation is valid for turbulent flow in cylindrical smooth pipe.
</p>
</html>",           revisions="<html>
<p>
<ul>
<li>
February 14, 2014, by Michael Wetter:<br/>
Removed unused input <code>rBor</code>.
Revised documentation.
</li>
<li>
January 24, 2014, by Michael Wetter:<br/>
Revised implementation. 
Changed <code>cpFluid</code> to <code>cpMed</code> to use consistent notation.
Added regularization for computation of convective heat transfer coefficient to
avoid an event and a non-differentiability.
</li>
<li>
January 23, 2014, by Damien Picard:<br/>
First implementation.
</li>
</ul>
</p>
</html>"));
          end convectionResistance;

          model SingleBoreHoleUTube "Single U-tube borehole heat exchanger"
            extends IBPSA.Fluid.Interfaces.PartialTwoPortInterface;

            extends IBPSA.Fluid.Interfaces.TwoPortFlowResistanceParameters;
            extends IBPSA.Fluid.Interfaces.LumpedVolumeDeclarations;

            InternalHEXUTube intHex[borFieDat.conDat.nVer](
              redeclare each final package Medium = Medium,
              each final from_dp1=from_dp,
              each final from_dp2=from_dp,
              each final linearizeFlowResistance1=linearizeFlowResistance,
              each final linearizeFlowResistance2=linearizeFlowResistance,
              each final deltaM1=deltaM,
              each final deltaM2=deltaM,
              each final energyDynamics=energyDynamics,
              each final massDynamics=massDynamics,
              each final T_start=T_start,
              each final dynFil=dynFil,
              each final mSenFac=mSenFac,
              final dp1_nominal={if i == 1 then dp_nominal else 0 for i in 1:borFieDat.conDat.nVer},
              each final dp2_nominal=0,
              each final m1_flow_nominal=m_flow_nominal,
              each final m2_flow_nominal=m_flow_nominal,
              each final borFieDat=borFieDat,
              each final allowFlowReversal1=allowFlowReversal,
              each final allowFlowReversal2=allowFlowReversal,
              each final show_T=show_T,
              each final p1_start=p_start,
              each final p2_start=p_start) "Discretized borehole segments"
              annotation (Placement(transformation(extent={{-10,-10},{10,10}})));

            parameter Boolean dynFil=true
                "Set to false to remove the dynamics of the filling material"
                annotation (Dialog(tab="Dynamics"));
            parameter
              IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.BorefieldData.Template
              borFieDat "Borefield parameters"
              annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
            Modelica.SIunits.Temperature TWallAve "Average borehole wall temperature";
            Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_wall[borFieDat.conDat.nVer]
              "Thermal connection for borehole wall"
              annotation (Placement(transformation(extent={{-10,90},{10,110}})));
          equation
            TWallAve =sum(intHex[:].port_wall.T)/borFieDat.conDat.nVer;

            connect(port_a, intHex[1].port_a1) annotation (Line(
                points={{-100,5.55112e-016},{-52,5.55112e-016},{-52,6.36364},{-10,6.36364}},
                color={0,127,255},
                smooth=Smooth.None));

            connect(port_b, intHex[1].port_b2) annotation (Line(
                points={{100,5.55112e-016},{28,5.55112e-016},{28,-40},{-32,-40},{-32,
                    -4.54545},{-10,-4.54545}},
                color={0,127,255},
                smooth=Smooth.None));
            connect(intHex[borFieDat.conDat.nVer].port_b1, intHex[borFieDat.conDat.nVer].port_a2)
              annotation (Line(
                points={{10,6},{20,6},{20,-6},{10,-6}},
                color={0,127,255},
                smooth=Smooth.None));
            for i in 1:borFieDat.conDat.nVer - 1 loop
              connect(intHex[i].port_b1, intHex[i + 1].port_a1) annotation (Line(
                  points={{10,6.36364},{10,20},{-10,20},{-10,6.36364}},
                  color={0,127,255},
                  smooth=Smooth.None));
              connect(intHex[i].port_a2, intHex[i + 1].port_b2) annotation (Line(
                  points={{10,-4.54545},{10,-20},{-10,-20},{-10,-4.54545}},
                  color={0,127,255},
                  smooth=Smooth.None));
            end for;
            connect(intHex.port_wall, port_wall)
              annotation (Line(points={{0,10},{0,10},{0,100}}, color={191,0,0}));
            annotation (
              Dialog(group="Borehole"),
              Dialog(group="Borehole"),
              defaultComponentName="borehole",
              Icon(coordinateSystem(
                  preserveAspectRatio=true,
                  extent={{-100,-100},{100,100}},
                  grid={2,2},
                  initialScale=0.5), graphics={
                  Rectangle(
                    extent={{-68,76},{72,-84}},
                    lineColor={0,0,255},
                    pattern=LinePattern.None,
                    fillColor={95,95,95},
                    fillPattern=FillPattern.Solid),
                  Rectangle(
                    extent={{-60,-56},{64,-64}},
                    lineColor={0,0,255},
                    pattern=LinePattern.None,
                    fillColor={0,0,0},
                    fillPattern=FillPattern.Solid),
                  Rectangle(
                    extent={{-60,54},{64,50}},
                    lineColor={0,0,255},
                    pattern=LinePattern.None,
                    fillColor={0,0,0},
                    fillPattern=FillPattern.Solid),
                  Rectangle(
                    extent={{-60,2},{64,-4}},
                    lineColor={0,0,255},
                    pattern=LinePattern.None,
                    fillColor={0,0,0},
                    fillPattern=FillPattern.Solid),
                  Rectangle(
                    extent={{56,88},{48,-92}},
                    lineColor={0,0,255},
                    pattern=LinePattern.None,
                    fillColor={0,0,255},
                    fillPattern=FillPattern.Solid),
                  Rectangle(
                    extent={{-52,-92},{-44,88}},
                    lineColor={0,0,255},
                    pattern=LinePattern.None,
                    fillColor={0,0,255},
                    fillPattern=FillPattern.Solid),
                  Rectangle(
                    extent={{-70,76},{-60,-84}},
                    lineColor={0,0,0},
                    fillColor={192,192,192},
                    fillPattern=FillPattern.Backward),
                  Rectangle(
                    extent={{64,76},{74,-84}},
                    lineColor={0,0,0},
                    fillColor={192,192,192},
                    fillPattern=FillPattern.Backward),
                  Rectangle(
                    extent={{-50,-84},{56,-92}},
                    lineColor={0,0,255},
                    pattern=LinePattern.None,
                    fillColor={0,0,255},
                    fillPattern=FillPattern.Solid)}),
              Diagram(coordinateSystem(
                  preserveAspectRatio=false,
                  extent={{-100,-100},{100,100}},
                  grid={2,2},
                  initialScale=0.5), graphics={Text(
                    extent={{60,72},{84,58}},
                    lineColor={0,0,255},
                    textString="")}),
              Documentation(info="<html>
<p>
Model of a single U-tube borehole heat exchanger. 
The borehole heat exchanger is vertically discretized into <i>n<sub>seg</sub></i>
elements of height <i>h=h<sub>Bor</sub>&frasl;n<sub>seg</sub></i>.
each final segment contains a model for the heat transfer in the borehole, 
for heat transfer in the soil and for the far-field boundary condition.
</p>
<p>
The heat transfer in the borehole is computed using a convective heat transfer coefficient
that depends on the fluid velocity, a heat resistance between the two pipes, and
a heat resistance between the pipes and the circumference of the borehole.
The heat capacity of the fluid, and the heat capacity of the grout, is taken into account.
All thermal mass is assumed to be at the two bulk temperatures of the down-flowing 
and up-flowing fluid.
</p>
<p>
The heat transfer in the soil is computed using transient heat conduction in cylindrical
coordinates for the spatial domain <i>r<sub>bor</sub> &le; r &le; r<sub>ext</sub></i>. 
In the radial direction, the spatial domain is discretized into 
<i>n<sub>hor</sub></i> segments with uniform material properties.
Thermal properties can be specified separately for each final horizontal layer.
The vertical heat flow is assumed to be zero, and there is assumed to be 
no ground water flow. 
</p>
<p>
The far-field temperature, i.e., the temperature at the radius 
<i>r<sub>ext</sub></i>, is kept constant because this model is only use to compute the short-term
temperature response of the borehole.
</p>

<h4>Implementation</h4>
<p>
each final horizontal layer is modeled using an instance of
<a href=\"modelica://IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Borefield2.BaseClasses.BoreHoles.BaseClasses.BoreHoleSegmentFourPort\">
IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Borefield2.BaseClasses.BoreHoles.BaseClasses.BoreHoleSegmentFourPort</a>.
This model is composed of the model
<a href=\"modelica://IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Borefield2.BaseClasses.BoreHoles.BaseClasses.SingleUTubeInternalHEX\">
IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Borefield2.BaseClasses.BoreHoles.BaseClasses.SingleUTubeInternalHEX</a> which computes
the heat transfer in the pipes and the borehole filling, and
of the model
<a href=\"modelica://IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Borefield2.BaseClasses.BoreHoles.BaseClasses.CylindricalGroundLayer\">
IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Borefield2.BaseClasses.BoreHoles.BaseClasses.CylindricalGroundLayer</a> which computes
the heat transfer in the soil.
</p>
</html>",           revisions="<html>
<ul>
<li>
July 2014, by Damien Picard:<br>
First implementation.
</li>
</ul>
</html>"));
          end SingleBoreHoleUTube;

          model CylindricalGroundLayerNAL
            "Heat conduction in a cylinder using the radial discretization as advised by Eskilson"

            parameter IBPSA.Fluid.HeatExchangers.GroundHeatExchangers.Data.SoilData.Template soiDat
              annotation (Placement(transformation(extent={{-100,-100},{-80,-80}})));
            parameter Modelica.SIunits.Height h "Height of the cylinder";
            parameter Modelica.SIunits.Radius r_a "Internal radius";
            parameter Modelica.SIunits.Radius r_b "External radius";
            parameter Integer nSta(min=4) = 10 "Number of state variables";
            parameter Modelica.SIunits.Temperature TInt_start=293.15
              "Initial temperature at port_a, used if steadyStateInitial = false"
              annotation (Dialog(group="Initialization", enable=not steadyStateInitial));
            parameter Modelica.SIunits.Temperature TExt_start=293.15
              "Initial temperature at port_b, used if steadyStateInitial = false"
              annotation (Dialog(group="Initialization", enable=not steadyStateInitial));
            parameter Boolean steadyStateInitial=false
              "true initializes dT(0)/dt=0, false initializes T(0) at fixed temperature using T_a_start and T_b_start"
              annotation (Dialog(group="Initialization"), Evaluate=true);

            parameter Real gridFac(min=1) = 2 "Grid factor for spacing";

            Modelica.SIunits.TemperatureDifference dT "port_a.T - port_b.T";

            Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_a port_a(T(start=TInt_start))
              "Heat port at surface a" annotation (Placement(transformation(extent={{-110,
                      -10},{-90,10}}, rotation=0)));
            Modelica.Thermal.HeatTransfer.Interfaces.HeatPort_b port_b(T(start=TExt_start))
              "Heat port at surface b" annotation (Placement(transformation(extent={{90,-10},
                      {110,10}},rotation=0)));

            parameter Modelica.SIunits.Radius r[nSta + 1](each fixed=false)
              "Radius to the boundary of the i-th domain";

            Modelica.Thermal.HeatTransfer.Components.ThermalResistor[nSta+1] Rsoil(R = {1/G[i] for i in 1:nSta+1})
              "Borehole resistance" annotation (Placement(transformation(
                  extent={{12,-12},{-12,12}},
                  rotation=180,
                  origin={0,0})));
            Modelica.Thermal.HeatTransfer.Components.HeatCapacitor[nSta] Csoil(C=C)
              annotation (Placement(transformation(extent={{-12,42},{12,66}})));


        protected
            parameter Modelica.SIunits.Radius rC[nSta] = {(r[i] + r[i + 1])/2 for i in 1:nSta}
              "Radius to the center of the i-th domain";

            final parameter Modelica.SIunits.SpecificHeatCapacity c=soiDat.c
              "Specific heat capacity";
            final parameter Modelica.SIunits.ThermalConductivity k=soiDat.k
              "Thermal conductivity of the material";
            final parameter Modelica.SIunits.Density d=soiDat.d
              "Density of the material";

            parameter Modelica.SIunits.ThermalConductance G[nSta + 1] =  cat(1, {2*Modelica.Constants.pi*k*h/Modelica.Math.log(rC[1]/r_a)}, {2*Modelica.Constants.pi*k*h/Modelica.Math.log(rC[i]/rC[i - 1]) for i in 2:nSta}, {2*Modelica.Constants.pi*k*h/Modelica.Math.log(r_b/rC[nSta])})
              "Heat conductance between the temperature nodes";
            parameter Modelica.SIunits.HeatCapacity C[nSta] = {(d*Modelica.Constants.pi*c*h*((r[i + 1])^2 - (r[i])^2)) for i in 1:nSta}
              "Heat capacity of each state";

            parameter Real gridFac_sum(fixed=false);
            parameter Real gridFac_sum_old(fixed=false);

          initial algorithm
            for i in 0:nSta - 3 - 1 loop
              if i == 0 then
                gridFac_sum := gridFac^i;
                gridFac_sum_old := gridFac_sum;
              else
                gridFac_sum := gridFac_sum_old + gridFac^i;
                gridFac_sum_old := gridFac_sum;
              end if;
            end for;

          initial equation
            assert(r_a < r_b, "Error: Model requires r_a < r_b.");
            assert(0 < r_a, "Error: Model requires 0 < r_a.");

            // ****************** Comments *********************:
            // The layer is divided into nSta segments. The radius of each segment increase exponentially with base 'griFac':
            // r_b - r_a = sum( a * griFac^k - a , k=2..nSta) => from this we find a = (r_b - r_a)/(griFac^(n+1) - griFac)
            // Using this a, we can now find the r[i]:
            // r[1] := r_a
            // r[i] = r[i-1] + a * griFac^i = r[i-1] + griFac^i * (r_b - r_a) / (griFac^(n+1) - griFac)
            //
            // This can also be writing as:
            // r[i]= r[i-1] + ( r_b - r_a)  * (1-griFac)/(1-griFac^(nSta)) * griFac^(i-2);

            r[1] = r_a;
            r[2] = r_a + sqrt(k/c/d*60) "eskilson minimum length";
            r[3] = r_a + 2*sqrt(k/c/d*60);
            r[4] = r_a + 3*sqrt(k/c/d*60);

            for i in 5:nSta + 1 loop
              r[i] = r[i - 1] + (r_b - r[4])/gridFac_sum*gridFac^(i - 5);
            end for;

            //Keep initial equation till here
            assert(abs(r[nSta + 1] - r_b) < 1E-3,
              "Error: Wrong computation of radius. r[nSta+1]=" + String(r[nSta + 1]));

            // The initialization is only done for materials that store energy.
            if not soiDat.steadyState then
              if steadyStateInitial then
                der(Csoil.port.T) = zeros(nSta);
              else
                for i in 1:nSta loop
                  Csoil[i].port.T = TInt_start + (TExt_start - TInt_start)/Modelica.Math.log(r_b/r_a)
                    *Modelica.Math.log(rC[i]/r_a);
                end for;
              end if;
            end if;

          equation
            dT = port_a.T - port_b.T;
            connect(port_a, Rsoil[1].port_a)
              annotation (Line(points={{-100,0},{-56,0},{-12,0}}, color={191,0,0}));
            connect(Rsoil[nSta+1].port_b, port_b)
              annotation (Line(points={{12,0},{100,0},{100,0}}, color={191,0,0}));
            for i in 1:nSta loop
              connect(Rsoil[i].port_b, Rsoil[i+1].port_a);
              connect(Csoil[i].port, Rsoil[i].port_b) annotation (Line(points={{0,42},{0,32},
                    {20,32},{20,0},{12,0}}, color={191,0,0}));
            end for;
             annotation (
              Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
                      100,100}})),
              Icon(coordinateSystem(preserveAspectRatio=false,extent={{-100,-100},{100,100}}),
                              graphics={
                  Rectangle(
                    extent={{-94,4},{92,-4}},
                    lineColor={0,0,0},
                    fillColor={191,0,0},
                    fillPattern=FillPattern.Solid),
                  Polygon(
                    points={{12,8},{14,8},{16,4},{18,-2},{18,-6},{16,-12},{10,-16},{6,-20},
                        {-2,-22},{-6,-18},{-12,-12},{-14,-2},{-12,4},{-10,8},{-8,10},{-6,
                        12},{-2,14},{2,14},{8,12},{12,8}},
                    lineColor={0,0,0},
                    smooth=Smooth.None,
                    fillColor={215,215,215},
                    fillPattern=FillPattern.Solid),
                  Polygon(
                    points={{-6,-16},{2,-18},{8,-16},{14,-14},{10,-16},{6,-20},{-2,-22},{
                        -8,-20},{-12,-12},{-14,-2},{-12,4},{-10,8},{-8,10},{-10,0},{-10,-8},
                        {-6,-16}},
                    lineColor={0,0,0},
                    smooth=Smooth.None,
                    fillColor={135,135,135},
                    fillPattern=FillPattern.Solid),
                  Text(
                    extent={{-110,-74},{-26,-86}},
                    lineColor={0,0,255},
                    textString="%r_a"),
                  Text(
                    extent={{-22,-62},{20,-76}},
                    lineColor={0,0,255},
                    textString="%nSta"),
                  Text(
                    extent={{16,-76},{102,-88}},
                    lineColor={0,0,255},
                    textString="%r_b"),
                  Polygon(
                    points={{-50,60},{-38,34},{-32,0},{-36,-30},{-50,-60},{-62,-60},{-48,
                        -30},{-44,0},{-50,34},{-62,60},{-50,60}},
                    lineColor={0,0,0},
                    smooth=Smooth.None,
                    fillPattern=FillPattern.Backward,
                    fillColor={175,175,175}),
                  Polygon(
                    points={{52,60},{64,34},{70,0},{66,-30},{52,-60},{40,-60},{54,-30},{
                        58,0},{52,34},{40,60},{52,60}},
                    lineColor={0,0,0},
                    smooth=Smooth.None,
                    fillPattern=FillPattern.Backward,
                    fillColor={175,175,175}),
                  Text(
                    extent={{-100,100},{100,60}},
                    lineColor={0,0,255},
                    textString="%name")}),
              defaultComponentName="lay",
              Documentation(info="<html>
<p>
Model for radial heat transfer in a hollow cylinder.
</p>
<p>
If the heat capacity of the material is non-zero, then this model computes transient heat conduction, i.e., it
computes a numerical approximation to the solution of the heat equation
</p>
<p align=\"center\" style=\"font-style:italic;\">
   &rho; c ( &part; T(r,t) &frasl; &part;t ) = 
    k ( &part;&sup2; T(r,t) &frasl; &part;r&sup2; + 1 &frasl; r &nbsp;  &part; T(r,t) &frasl; &part;r ),
</p>
<p>
where 
<i>&rho;</i>
is the mass density,
<i>c</i>
is the specific heat capacity per unit mass,
<i>T</i>
is the temperature at location <i>r</i> and time <i>t</i> and
<i>k</i> is the heat conductivity. 
At the locations <i>r=r<sub>a</sub></i> and <i>r=r<sub>b</sub></i>, 
the temperature and heat flow rate are equal to the 
temperature and heat flow rate of the heat ports.
</p>
<p>
If the heat capacity of the material is set to zero, then steady-state heat flow is computed using
</p>
<p align=\"center\" style=\"font-style:italic;\">
   Q = 2 &pi; k (T<sub>a</sub>-T<sub>b</sub>)&frasl; ln(r<sub>a</sub> &frasl; r<sub>b</sub>),
</p>
<p>
where
<i>r<sub>a</sub></i> is the internal radius,
<i>r<sub>b</sub></i> is the external radius,
<i>T<sub>a</sub></i> is the temperature at port a and
<i>T<sub>b</sub></i> is the temperature at port b.
</p>
<h4>Implementation</h4>
<p>
To spatially discretize the heat equation, the construction is 
divided into compartments with <code>material.nSta &ge; 1</code> state variables. 
The state variables are connected to each other through thermal conductors. 
There is also a thermal conductor
between the surfaces and the outermost state variables. Thus, to obtain
the surface temperature, use <code>port_a.T</code> (or <code>port_b.T</code>)
and not the variable <code>T[1]</code>.
</p>
</html>",           revisions="<html>
<ul>
<li>
Januari, 2014, by Damien Picard:<br/>
Modify the discretization of the cilindrical layer so that the first three layers have an equal thickness the following an exponentionally growing thickness.
This follows the guidelines of Eskilson (P. Eskilson. Thermal analysis of heat extraction
boreholes. PhD thesis, Dep. of Mathematical
Physics, University of Lund, Sweden, 1987).
</li>
<li>
March 9, 2012, by Michael Wetter:<br/>
Removed protected variable <code>der_T</code> as it is not required.
</li>
<li>
April 14 2011, by Pierre Vigouroux:<br/>
First implementation.
</li>
</ul>
</html>"));
          end CylindricalGroundLayerNAL;
        end BaseClasses;
      end GroundHeatExchangers;
    end HeatExchangers;

    package HeatPumps
    extends Modelica.Icons.VariantsPackage;
      model HeatPump "A heat pump model for optimization"

        replaceable package Medium1 =
            Modelica.Media.Interfaces.PartialMedium "Medium through the sink side (condenser)"
            annotation (choicesAllMatching = true);

                replaceable package Medium2 =
          Modelica.Media.Interfaces.PartialMedium
        "Medium through the source side (evaporator)" annotation (
          choicesAllMatching=true);

        Modelica.Blocks.Sources.RealExpression COP(y=COP_expr)
          "COP expression of the heat pump"
          annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
        IDEAS.Fluid.HeatExchangers.PrescribedOutlet HP_con(
          allowFlowReversal=false,
          energyDynamics=Modelica.Fluid.Types.Dynamics.SteadyState,
          use_TSet=true,
          use_X_wSet=false,
          redeclare package Medium = Medium1,
          m_flow_nominal=m1_flow_nominal,
          dp_nominal=dp1_nominal)
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
          initType=Modelica.Blocks.Types.Init.SteadyState,
          redeclare package Medium = Medium2,
          m_flow_nominal=m2_flow_nominal,
          tau=0)
          annotation (Placement(transformation(extent={{70,-70},{50,-50}})));
        IDEAS.Fluid.Sensors.TemperatureTwoPort T_con_in(
          allowFlowReversal=false,
          initType=Modelica.Blocks.Types.Init.SteadyState,
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
          initType=Modelica.Blocks.Types.Init.SteadyState,
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

        Modelica.Blocks.Interfaces.RealOutput COP_expr=6.4 - 0.16*(T_con_in.T - 298.15)
             + 0.1*(T_eva_in.T - 288.15) "COP expression of the heat pump"
          annotation (Dialog(tab="Advanced"));
        Modelica.Blocks.Interfaces.RealInput Tcon_out
          "outlet condenser temperature signal"
          annotation (Placement(transformation(extent={{-120,70},{-80,110}})));
        Modelica.Blocks.Sources.RealExpression Wcomp(y=HP_con.Q_flow/COP.y)
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
        connect(Tcon_out, HP_con.TSet) annotation (Line(points={{-100,90},{-40,90},{-40,
                68},{-12,68}}, color={0,0,127}));
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
        connect(HP_con.Q_flow, Q_con) annotation (Line(points={{11,68},{12,68},{12,84},
                {110,84}}, color={0,0,127}));
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
      end HeatPump;
    end HeatPumps;
  annotation (Icon(graphics={
          Polygon(points={{-70,26},{68,-44},{68,26},{2,-10},{-70,-42},{-70,26}},
              lineColor={0,0,0}),
          Rectangle(
            extent={{-18,50},{22,42}},
            lineColor={0,0,0},
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Line(points={{2,42},{2,-10}})}));
  end Fluid;

  package Media "Media that are tailored for optimization"
    package DryAir "Package with dry air model"
      extends BuildingMpc.Media.PartialSimpleMedium(
      mediumName="Air",
      cp_const=1005,
      cv_const=cp_const/1.4,
      d_const=1.2,
      eta_const=1.82e-5,
      lambda_const=0.026,
      a_const=331,
      T_min=240,
      T_max=400,
      T0=reference_T,
      MM_const=0.0289651159,
      reference_X={1},
      reference_T=273.15,
      reference_p=101325,
      AbsolutePressure(start=p_default),
      Temperature(start=T_default),
      extraPropertiesNames=fill("CO2", if useCO2 then 1 else 0));
      constant Boolean useCO2 = false;

      extends Modelica.Icons.Package;

      function enthalpyOfCondensingGas
        "Enthalpy of steam per unit mass of steam"
        extends Modelica.Icons.Function;
        input Temperature T "temperature";
        output SpecificEnthalpy h "steam enthalpy";
      algorithm
        h := 0;
        annotation(smoothOrder=5,
        Inline=true);
      end enthalpyOfCondensingGas;
      annotation(preferredView="info", Documentation(info="<html>
<p>
Very simple air medium that does not cause linear algebraic loops in JModelica.
</p>
</html>",     revisions="<html>
<ul>
<li>
2017, by Filip Jorissen:<br/>
First implementation.
</li>
</ul>
</html>"),
        Icon(graphics={
            Ellipse(
              extent={{-78,78},{-34,34}},
              lineColor={0,0,0},
              fillPattern=FillPattern.Sphere,
              fillColor={120,120,120}),
            Ellipse(
              extent={{-18,86},{26,42}},
              lineColor={0,0,0},
              fillPattern=FillPattern.Sphere,
              fillColor={120,120,120}),
            Ellipse(
              extent={{48,58},{92,14}},
              lineColor={0,0,0},
              fillPattern=FillPattern.Sphere,
              fillColor={120,120,120}),
            Ellipse(
              extent={{-22,32},{22,-12}},
              lineColor={0,0,0},
              fillPattern=FillPattern.Sphere,
              fillColor={120,120,120}),
            Ellipse(
              extent={{36,-32},{80,-76}},
              lineColor={0,0,0},
              fillPattern=FillPattern.Sphere,
              fillColor={120,120,120}),
            Ellipse(
              extent={{-36,-30},{8,-74}},
              lineColor={0,0,0},
              fillPattern=FillPattern.Sphere,
              fillColor={120,120,120}),
            Ellipse(
              extent={{-90,-6},{-46,-50}},
              lineColor={0,0,0},
              fillPattern=FillPattern.Sphere,
              fillColor={120,120,120})}));
    end DryAir;

    partial package PartialSimpleMedium "Medium model with linear dependency of u, h from temperature. All other quantities, especially density, are constant."

      extends Modelica.Media.Interfaces.PartialPureSubstance(final ThermoStates=
            Modelica.Media.Interfaces.Choices.IndependentVariables.pT, final
          singleState=true);

      constant SpecificHeatCapacity cp_const
        "Constant specific heat capacity at constant pressure";
      constant SpecificHeatCapacity cv_const
        "Constant specific heat capacity at constant volume";
      constant Density d_const "Constant density";
      constant DynamicViscosity eta_const "Constant dynamic viscosity";
      constant ThermalConductivity lambda_const "Constant thermal conductivity";
      constant VelocityOfSound a_const "Constant velocity of sound";
      constant Temperature T_min "Minimum temperature valid for medium model";
      constant Temperature T_max "Maximum temperature valid for medium model";
      constant Temperature T0=reference_T "Zero enthalpy temperature";
      constant MolarMass MM_const "Molar mass";

      constant FluidConstants[nS] fluidConstants "Fluid constants";

      redeclare record extends ThermodynamicState "Thermodynamic state"
        AbsolutePressure p "Absolute pressure of medium";
        Temperature T "Temperature of medium";
      end ThermodynamicState;

      redeclare replaceable model extends BaseProperties(T(stateSelect=if
              preferredMediumStates then StateSelect.prefer else StateSelect.default),
          p(stateSelect=if preferredMediumStates then StateSelect.prefer else
              StateSelect.default)) "Base properties"
      equation
        assert(T >= T_min and T <= T_max, "
Temperature T (= "   + String(T) + " K) is not
in the allowed range ("   + String(T_min) + " K <= T <= " + String(T_max) + " K)
required from medium model \""   + mediumName + "\".
");

        // h = cp_const*(T-T0);
        h = specificEnthalpy_pTX(
                p,
                T,
                X);
        T=T0 + u/cv_const;
        //u = cv_const*(T - T0);
        d = d_const;
        R = 0;
        MM = MM_const;
        state.T = T;
        state.p = p;
        annotation (Documentation(info="<html>
<p>
This is the most simple incompressible medium model, where
specific enthalpy h and specific internal energy u are only
a function of temperature T and all other provided medium
quantities are assumed to be constant.
Note that the (small) influence of the pressure term p/d is neglected.
</p>
</html>"));
      end BaseProperties;

      redeclare function setState_pTX
        "Return thermodynamic state from p, T, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input Temperature T "Temperature";
        input MassFraction X[:]=reference_X "Mass fractions";
        output ThermodynamicState state "Thermodynamic state record";
      algorithm
        state := ThermodynamicState(p=p, T=T);
      end setState_pTX;

      redeclare function setState_phX
        "Return thermodynamic state from p, h, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEnthalpy h "Specific enthalpy";
        input MassFraction X[:]=reference_X "Mass fractions";
        output ThermodynamicState state "Thermodynamic state record";
      algorithm
        state := ThermodynamicState(p=p, T=T0 + h/cp_const);
      end setState_phX;

      redeclare replaceable function setState_psX
        "Return thermodynamic state from p, s, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEntropy s "Specific entropy";
        input MassFraction X[:]=reference_X "Mass fractions";
        output ThermodynamicState state "Thermodynamic state record";
      algorithm
        state := ThermodynamicState(p=p, T=Modelica.Math.exp(s/cp_const +
          Modelica.Math.log(reference_T)))
          "Here the incompressible limit is used, with cp as heat capacity";
      end setState_psX;

      redeclare function setState_dTX
        "Return thermodynamic state from d, T, and X or Xi"
        extends Modelica.Icons.Function;
        input Density d "Density";
        input Temperature T "Temperature";
        input MassFraction X[:]=reference_X "Mass fractions";
        output ThermodynamicState state "Thermodynamic state record";
      algorithm
        assert(false,
          "Pressure can not be computed from temperature and density for an incompressible fluid!");
      end setState_dTX;

      redeclare function extends setSmoothState
        "Return thermodynamic state so that it smoothly approximates: if x > 0 then state_a else state_b"
      algorithm
        state := ThermodynamicState(p=Modelica.Media.Common.smoothStep(
            x,
            state_a.p,
            state_b.p,
            x_small), T=Modelica.Media.Common.smoothStep(
            x,
            state_a.T,
            state_b.T,
            x_small));
      end setSmoothState;

      redeclare function extends dynamicViscosity "Return dynamic viscosity"

      algorithm
        eta := eta_const;
      end dynamicViscosity;

      redeclare function extends thermalConductivity
        "Return thermal conductivity"

      algorithm
        lambda := lambda_const;
      end thermalConductivity;

      redeclare function extends pressure "Return pressure"

      algorithm
        p := state.p;
      end pressure;

      redeclare function extends temperature "Return temperature"

      algorithm
        T := state.T;
      end temperature;

      redeclare function extends density "Return density"

      algorithm
        d := d_const;
      end density;

      redeclare function extends specificEnthalpy "Return specific enthalpy"

      algorithm
        h := cp_const*(state.T - T0);
      end specificEnthalpy;

      redeclare function extends specificHeatCapacityCp
        "Return specific heat capacity at constant pressure"

      algorithm
        cp := cp_const;
      end specificHeatCapacityCp;

      redeclare function extends specificHeatCapacityCv
        "Return specific heat capacity at constant volume"

      algorithm
        cv := cv_const;
      end specificHeatCapacityCv;

      redeclare function extends isentropicExponent "Return isentropic exponent"

      algorithm
        gamma := cp_const/cv_const;
      end isentropicExponent;

      redeclare function extends velocityOfSound "Return velocity of sound"

      algorithm
        a := a_const;
      end velocityOfSound;

      redeclare function specificEnthalpy_pTX
        "Return specific enthalpy from p, T, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input Temperature T "Temperature";
        input MassFraction X[nX] "Mass fractions";
        output SpecificEnthalpy h "Specific enthalpy";
      algorithm
        h := cp_const*(T - T0);
        annotation (Documentation(info="<html>
<p>
This function computes the specific enthalpy of the fluid, but neglects the (small) influence of the pressure term p/d.
</p>
</html>"));
      end specificEnthalpy_pTX;

      redeclare function temperature_phX
        "Return temperature from p, h, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEnthalpy h "Specific enthalpy";
        input MassFraction X[nX] "Mass fractions";
        output Temperature T "Temperature";
      algorithm
        T := T0 + h/cp_const;
      end temperature_phX;

      redeclare function density_phX "Return density from p, h, and X or Xi"
        extends Modelica.Icons.Function;
        input AbsolutePressure p "Pressure";
        input SpecificEnthalpy h "Specific enthalpy";
        input MassFraction X[nX] "Mass fractions";
        output Density d "Density";
      algorithm
        d := density(setState_phX(
                p,
                h,
                X));
      end density_phX;

      redeclare function extends specificInternalEnergy
        "Return specific internal energy"
        extends Modelica.Icons.Function;
      algorithm
        //  u := cv_const*(state.T - T0) - reference_p/d_const;
        u := cv_const*(state.T - T0);
        annotation (Documentation(info="<html>
<p>
This function computes the specific internal energy of the fluid, but neglects the (small) influence of the pressure term p/d.
</p>
</html>"));
      end specificInternalEnergy;

      redeclare function extends specificEntropy "Return specific entropy"
        extends Modelica.Icons.Function;
      algorithm
        s := cv_const*Modelica.Math.log(state.T/T0);
      end specificEntropy;

      redeclare function extends specificGibbsEnergy
        "Return specific Gibbs energy"
        extends Modelica.Icons.Function;
      algorithm
        g := specificEnthalpy(state) - state.T*specificEntropy(state);
      end specificGibbsEnergy;

      redeclare function extends specificHelmholtzEnergy
        "Return specific Helmholtz energy"
        extends Modelica.Icons.Function;
      algorithm
        f := specificInternalEnergy(state) - state.T*specificEntropy(state);
      end specificHelmholtzEnergy;

      redeclare function extends isentropicEnthalpy "Return isentropic enthalpy"
      algorithm
        h_is := cp_const*(temperature(refState) - T0);
      end isentropicEnthalpy;

      redeclare function extends isobaricExpansionCoefficient
        "Returns overall the isobaric expansion coefficient beta"
      algorithm
        beta := 0.0;
      end isobaricExpansionCoefficient;

      redeclare function extends isothermalCompressibility
        "Returns overall the isothermal compressibility factor"
      algorithm
        kappa := 0;
      end isothermalCompressibility;

      redeclare function extends density_derp_T
        "Returns the partial derivative of density with respect to pressure at constant temperature"
      algorithm
        ddpT := 0;
      end density_derp_T;

      redeclare function extends density_derT_p
        "Returns the partial derivative of density with respect to temperature at constant pressure"
      algorithm
        ddTp := 0;
      end density_derT_p;

      redeclare function extends density_derX
        "Returns the partial derivative of density with respect to mass fractions at constant pressure and temperature"
      algorithm
        dddX := fill(0, nX);
      end density_derX;

      redeclare function extends molarMass "Return the molar mass of the medium"
      algorithm
        MM := MM_const;
      end molarMass;

      annotation (Documentation(info="<html>
<p>
Partial medium that contains an explicit expression 
for the computation of T as a function of u.
This avoids algebraic loops in JModelica.
</p>
</html>"));
    end PartialSimpleMedium;
  end Media;

annotation (uses(
    Modelica(version="3.2.2"),
    IDEAS(version="1.0.0"),
    Solarwind(version="0.3"),
    IBPSA(version="1.0.0")));
end BuildingMpc;
