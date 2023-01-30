// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.core.JsonGenerator;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class ArmSolverIOKairos implements ArmSolverIO {
  private final StringPublisher configPublisher;
  private final StringPublisher requestPublisher;
  private final List<DoubleArraySubscriber> resultSubscribers = new ArrayList<>();

  private int parameterHash = 0;
  private boolean resultReceived = true;

  public ArmSolverIOKairos(int instanceCount) {
    // Create NT publishers and subscribers
    var kairosTable = NetworkTableInstance.getDefault().getTable("kairos");
    configPublisher =
        kairosTable.getStringTopic("config").publishEx("json", "{}", PubSubOption.periodic(0.0));
    requestPublisher =
        kairosTable
            .getStringTopic("request")
            .publishEx("json", "{}", PubSubOption.periodic(0.0), PubSubOption.keepDuplicates(true));
    for (int i = 0; i < instanceCount; i++) {
      resultSubscribers.add(
          kairosTable
              .getDoubleArrayTopic("result/" + Integer.toString(i))
              .subscribe(new double[] {}, PubSubOption.periodic(0.0)));
    }
  }

  public void updateInputs(ArmSolverIOInputs inputs) {
    if (resultReceived) return; // We already got a result

    // Check for new results
    for (var subscriber : resultSubscribers) {
      var value = subscriber.get();
      if (value.length > 0) {
        int resultParameterHash = (int) value[0];
        if (resultParameterHash == parameterHash) {
          // Result was received!
          resultReceived = true;
          inputs.parameterHash = resultParameterHash;
          inputs.totalTime = value[1];
          int pointCount = (value.length - 2) / 2;
          inputs.shoulderPoints = new double[pointCount];
          inputs.elbowPoints = new double[pointCount];
          for (int i = 0; i < pointCount; i++) {
            inputs.shoulderPoints[i] = value[i * 2 + 2];
            inputs.elbowPoints[i] = value[i * 2 + 3];
          }
        }
      }
    }
  }

  public void setConfig(String configJson) {
    configPublisher.set(configJson);
  }

  public void request(ArmTrajectory.Parameters parameters) {
    // Create JSON
    ByteArrayOutputStream stream = new ByteArrayOutputStream();
    JsonGenerator generator;
    try {
      generator = new JsonFactory().createGenerator(stream);
      generator.writeStartObject();
      generator.writeNumberField("hash", parameters.hashCode());

      generator.writeArrayFieldStart("initialJointPositions");
      generator.writeNumber(parameters.initialJointPositions().get(0, 0));
      generator.writeNumber(parameters.initialJointPositions().get(1, 0));
      generator.writeEndArray();

      generator.writeArrayFieldStart("finalJointPositions");
      generator.writeNumber(parameters.finalJointPositions().get(0, 0));
      generator.writeNumber(parameters.finalJointPositions().get(1, 0));
      generator.writeEndArray();

      generator.writeArrayFieldStart("initialJointVelocities");
      generator.writeNumber(parameters.initialJointVelocities().get(0, 0));
      generator.writeNumber(parameters.initialJointVelocities().get(1, 0));
      generator.writeEndArray();

      generator.writeArrayFieldStart("finalJointVelocities");
      generator.writeNumber(parameters.finalJointVelocities().get(0, 0));
      generator.writeNumber(parameters.finalJointVelocities().get(1, 0));
      generator.writeEndArray();

      generator.writeArrayFieldStart("constraintKeys");
      for (var constraintKey : parameters.constraintKeys()) {
        generator.writeString(constraintKey);
      }
      generator.writeEndArray();
      generator.writeEndObject();
      generator.close();
      stream.close();
    } catch (IOException e) {
      e.printStackTrace();
    }
    String requestJson = stream.toString();

    // Send request
    parameterHash = parameters.hashCode();
    resultReceived = false;
    requestPublisher.set(requestJson);
  }
}
