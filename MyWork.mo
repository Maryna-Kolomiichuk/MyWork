within ;
package MyWork "My description"

  package Tutorial1

    model SimplePendulumUnits
      constant Real g(unit="m/s2") = 9.81;
      parameter Real L(min=0,unit="m") = 1;
      Real Theta(start=0.1, fixed=true);
      Real ThetaDot;
    equation
    ThetaDot = der(Theta);
    der(ThetaDot) = - g/L*sin(Theta);
      annotation (experiment(StopTime=10, __Dymola_NumberOfIntervals=50000));
    end SimplePendulumUnits;

    model SimplePendulumSI
      constant Modelica.Units.SI.Acceleration g = 9.81;
      parameter Modelica.Units.SI.Length L(min=0) = 1;
      Modelica.Units.SI.Angle Theta(start=0.1, fixed=true);
      Modelica.Units.SI.AngularVelocity ThetaDot;
    equation
    ThetaDot = der(Theta);
    der(ThetaDot) = - g/L*sin(Theta);
      annotation (experiment(StopTime=10, __Dymola_NumberOfIntervals=50000));
    end SimplePendulumSI;

    model SimplePendulumImport
      import Modelica.Units.SI;
      constant SI.Acceleration g = 9.81;
      parameter SI.Length L(min=0) = 1;
      SI.Angle Theta(start=0.1, fixed=true);
      SI.AngularVelocity ThetaDot;
    equation
    ThetaDot = der(Theta);
    der(ThetaDot) = - g/L*sin(Theta);
      annotation (experiment(StopTime=10, __Dymola_NumberOfIntervals=50000));
    end SimplePendulumImport;

    model SimplePendulumDescription "Model of a simple pendulum"
      import Modelica.Units.SI "Import to allow compact names";
      constant SI.Acceleration g=9.81 "Gravitational constant";
      parameter SI.Length L(min=0) = 1 "Length of the pendulum";
      /* List of variables */

      SI.Angle Theta(start=0.1, fixed=true) "Displacement angle";
      SI.AngularVelocity ThetaDot "Displacement velocity";
    equation
      ThetaDot = der(Theta) "Equation to allow second derivative";
      der(ThetaDot) = -g/L*sin(Theta);
    end SimplePendulumDescription;

    model SimplePendulum
      constant Real g = 9.81;
      parameter Real L = 1;
      Real Theta(start=0.1, fixed=true);
      Real ThetaDot;
    equation
    ThetaDot = der(Theta);
    der(ThetaDot) = - g/L*sin(Theta);
    end SimplePendulum;
  end Tutorial1;

  package Tutorial2

    model Motor
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=0.001)
        annotation (Placement(transformation(extent={{44,-10},{64,10}})));
      Modelica.Electrical.Analog.Basic.Resistor resistor(R=0.5)
        annotation (Placement(transformation(extent={{-38,10},{-18,30}})));
      Modelica.Electrical.Analog.Basic.Inductor inductor(L=0.05)
        annotation (Placement(transformation(extent={{-6,10},{14,30}})));
      Modelica.Electrical.Analog.Basic.Ground ground
        annotation (Placement(transformation(extent={{-60,-54},{-40,-34}})));
      Modelica.Electrical.Analog.Sources.SignalVoltage signalVoltage
        annotation (Placement(transformation(
            extent={{-10,10},{10,-10}},
            rotation=270,
            origin={-50,0})));
      Modelica.Electrical.Analog.Basic.RotationalEMF emf annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=0,
            origin={22,0})));
      Modelica.Blocks.Interfaces.RealInput realInput annotation (Placement(
            transformation(extent={{-140,-20},{-100,20}}), iconTransformation(
              extent={{-140,-20},{-100,20}})));
      Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b annotation (
          Placement(transformation(extent={{90,-10},{110,10}}),
            iconTransformation(extent={{90,-10},{110,10}})));
    equation
      connect(resistor.n, inductor.p)
        annotation (Line(points={{-18,20},{-6,20}}, color={0,0,255}));
      connect(resistor.p, signalVoltage.p) annotation (Line(points={{-38,20},{
              -50,20},{-50,10}}, color={0,0,255}));
      connect(ground.p, signalVoltage.n)
        annotation (Line(points={{-50,-34},{-50,-10}}, color={0,0,255}));
      connect(inductor.n, emf.p)
        annotation (Line(points={{14,20},{22,20},{22,10}}, color={0,0,255}));
      connect(emf.n, signalVoltage.n) annotation (Line(points={{22,-10},{22,-20},
              {-50,-20},{-50,-10}}, color={0,0,255}));
      connect(inertia.flange_a, emf.flange)
        annotation (Line(points={{44,0},{32,0}}, color={0,0,0}));
      connect(inertia.flange_b, flange_b)
        annotation (Line(points={{64,0},{100,0}}, color={0,0,0}));
      connect(signalVoltage.v, realInput)
        annotation (Line(points={{-62,0},{-120,0}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Motor;

    model MotorDrive
      Motor motor
        annotation (Placement(transformation(extent={{-12,32},{8,46}})));
      Modelica.Blocks.Sources.Step step(
        height=1,
        offset=0.5,
        startTime=5)
        annotation (Placement(transformation(extent={{-96,32},{-82,46}})));
      Modelica.Blocks.Continuous.PID PID(k=100, Ti=1)
        annotation (Placement(transformation(extent={{-40,32},{-26,46}})));
      Modelica.Blocks.Math.Feedback feedback
        annotation (Placement(transformation(extent={{-70,30},{-52,48}})));
      Modelica.Mechanics.Rotational.Components.IdealGear idealGear(ratio=100)
        annotation (Placement(transformation(extent={{20,30},{38,48}})));
      Modelica.Mechanics.Rotational.Components.Inertia inertia(J=5)
        annotation (Placement(transformation(extent={{54,30},{72,48}})));
      Modelica.Mechanics.Rotational.Sensors.AngleSensor angleSensor annotation
        (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={86,20})));
    equation
      connect(step.y, feedback.u1)
        annotation (Line(points={{-81.3,39},{-68.2,39}}, color={0,0,127}));
      connect(feedback.y, PID.u)
        annotation (Line(points={{-52.9,39},{-41.4,39}}, color={0,0,127}));
      connect(PID.y, motor.realInput)
        annotation (Line(points={{-25.3,39},{-14,39}}, color={0,0,127}));
      connect(motor.flange_b, idealGear.flange_a)
        annotation (Line(points={{8,39},{20,39}}, color={0,0,0}));
      connect(idealGear.flange_b, inertia.flange_a)
        annotation (Line(points={{38,39},{54,39}}, color={0,0,0}));
      connect(inertia.flange_b, angleSensor.flange)
        annotation (Line(points={{72,39},{86,39},{86,30}}, color={0,0,0}));
      connect(feedback.u2, angleSensor.phi) annotation (Line(points={{-61,31.8},
              {-61,0},{86,0},{86,9}}, color={0,0,127}));
      annotation (
        Icon(coordinateSystem(preserveAspectRatio=false)),
        Diagram(coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=10));
    end MotorDrive;
  end Tutorial2;
  annotation (uses(Modelica(version="4.0.0")));
end MyWork;
