package frc.robot.controller;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.controller.Controller;

public record ControllerTest() {
    @Test
    void shouldReturnValueWhenBadDeadzoneType() {
        // arrange
        Controller controller = new Controller(0);
        controller.setDeadzoneType(null);

        // act
        double value = controller.applyDeadzone(1, 1, false);

        // assert
        assertEquals(value, 0);
    }
}
