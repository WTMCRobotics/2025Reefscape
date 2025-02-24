package frc.robot.controller;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import frc.robot.controller.Controller.*;

public record ControllerTest() {
    @Test
    void shouldReturnValueWhenBadDeadzoneType() {
        // arrange
        Controller controller = new Controller(0);
        controller.setDeadzoneType(null);

        // act
        double value = controller.applyDeadzone(1, 1, false);

        // assert
        assertEquals(1, value);
    }

    @ParameterizedTest
    @CsvSource({"0.05,0", "0.1,0.1", "0.5,0.5"})
    void shouldReturnZeroWhenInsideSquareDeadzone(double input, double expected) {
        Controller controller = new Controller(0);
        controller.setDeadzoneType(Deadzone.SQUARE);

        double value = controller.applyDeadzone(input, 0.1, false);

        assertEquals(expected, value);
    }

    @ParameterizedTest
    @CsvSource({"0.05,0.0025", "0.1,0.01", "0.5,0.25"})
    void shouldReturnSquaredValueWhenSquaredProfile(double input, double expected) {
        Controller controller = new Controller(0);
        controller.setDeadzoneType(null);

        double value = controller.applyProfile(input, StickProfile.SQUARE);

        assertEquals(expected, value);
    }
}
