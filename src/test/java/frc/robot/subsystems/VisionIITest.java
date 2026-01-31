package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.when;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.verify;

import java.util.List;
import java.util.Optional;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.extension.ExtendWith;
import org.mockito.Mock;
import org.mockito.junit.jupiter.MockitoExtension;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * This is a test class for the VisionII subsystem.
 * It demonstrates how to use JUnit and Mockito to write unit tests for your code.
 * Unit tests are crucial for ensuring that individual components (or "units") of your software
 * work as expected in isolation.
 */
// @ExtendWith(MockitoExtension.class) tells JUnit to use the Mockito extension.
// This extension automatically initializes any fields annotated with @Mock.
@ExtendWith(MockitoExtension.class)
public class VisionIITest {

    /**
     * @Mock creates a mock object for the given class.
     * Mocks are "fake" objects that we can control in our tests.
     * Here, we are creating mocks for the dependencies of the VisionII class.
     * This allows us to test VisionII without needing real cameras or estimators.
     */
    @Mock
    private PhotonCamera mockLeftCam;
    @Mock
    private PhotonCamera mockRightCam;
    @Mock
    private PhotonPoseEstimator mockLeftEstimator;
    @Mock
    private PhotonPoseEstimator mockRightEstimator;

    // These are mock objects that will be returned by our other mocks.
    @Mock
    private PhotonPipelineResult mockPipelineResult;
    @Mock
    private EstimatedRobotPose mockEstimatedPose;

    // This is the object we are testing.
    private VisionII vision;

    /**
     * The @BeforeEach annotation marks a method that should be run before each test method.
     * This is a good place to set up the initial state for your tests, like
     * initializing the object under test.
     */
    @BeforeEach
    void setUp() {
        // We create a new VisionII instance before each test.
        // We use a special constructor that allows us to "inject" our mock dependencies.
        // This is a common pattern called "Dependency Injection" and is key for writing testable code.
        vision = new VisionII(mockLeftCam, mockRightCam, mockLeftEstimator, mockRightEstimator);
    }

    /**
     * This is our first test case. The @Test annotation tells JUnit that this is a test method.
     * A good test method name clearly describes what it is testing.
     * This test verifies that getVisionUpdates correctly processes results from both cameras
     * and returns the estimated poses.
     */
    @Test
    void testGetVisionUpdatesWithResults() {
        // --- ARRANGE ---
        // In the "Arrange" phase, we set up the state of our mocks.
        // We are defining what should happen when methods are called on our mock objects.

        // We tell our mock cameras to return a list containing our mock pipeline result
        // when getAllUnreadResults() is called.
        when(mockLeftCam.getAllUnreadResults()).thenReturn(List.of(mockPipelineResult));
        when(mockRightCam.getAllUnreadResults()).thenReturn(List.of(mockPipelineResult));

        // We tell our mock estimators to return an Optional containing our mock estimated pose
        // when estimateCoprocMultiTagPose() is called with the mock pipeline result.
        // The use of Optional.of() simulates the case where a pose is successfully estimated.
        when(mockLeftEstimator.estimateCoprocMultiTagPose(mockPipelineResult)).thenReturn(Optional.of(mockEstimatedPose));
        when(mockRightEstimator.estimateCoprocMultiTagPose(mockPipelineResult)).thenReturn(Optional.of(mockEstimatedPose));

        // --- ACT ---
        // In the "Act" phase, we call the method we want to test.
        List<EstimatedRobotPose> results = vision.getVisionUpdates();

        // --- ASSERT ---
        // In the "Assert" phase, we check if the result of the action is what we expected.

        // We expect to get two results (one from each camera).
        assertEquals(2, results.size(), "Should have received two pose estimates");
        // We also check that the results are the same mock objects we arranged to be returned.
        assertEquals(mockEstimatedPose, results.get(0), "The first pose should be the one from the right camera");
        assertEquals(mockEstimatedPose, results.get(1), "The second pose should be the one from the left camera");

        // Mockito's verify() method is used to check if certain methods on mock objects
        // were called during the test execution. This ensures that the interactions
        // with the dependencies happened as expected.
        verify(mockLeftCam).getAllUnreadResults();
        verify(mockRightCam).getAllUnreadResults();
        verify(mockLeftEstimator).estimateCoprocMultiTagPose(mockPipelineResult);
        verify(mockRightEstimator).estimateCoprocMultiTagPose(mockPipelineResult);
    }

    /**
     * This test case covers the scenario where the cameras have no new results.
     * We expect the getVisionUpdates method to return an empty list.
     */
    @Test
    void testGetVisionUpdatesWithNoResults() {
        // --- ARRANGE ---
        // We configure the mock cameras to return an empty list.
        when(mockLeftCam.getAllUnreadResults()).thenReturn(List.of());
        when(mockRightCam.getAllUnreadResults()).thenReturn(List.of());

        // --- ACT ---
        List<EstimatedRobotPose> results = vision.getVisionUpdates();

        // --- ASSERT ---
        // We assert that the returned list is empty.
        assertTrue(results.isEmpty(), "The list of poses should be empty");

        verify(mockLeftCam).getAllUnreadResults();
        verify(mockRightCam).getAllUnreadResults();
        verify(mockLeftEstimator, never()).estimateCoprocMultiTagPose(any());
        verify(mockRightEstimator, never()).estimateCoprocMultiTagPose(any());
    }

    /**
     * This test case covers the scenario where the cameras have results, but the
     * estimators are unable to calculate a pose (e.g., due to seeing ambiguous targets).
     * In this case, the estimator returns an empty Optional.
     */
    @Test
    void testGetVisionUpdatesWithNoEstimates() {
        // --- ARRANGE ---
        // The cameras have results...
        when(mockLeftCam.getAllUnreadResults()).thenReturn(List.of(mockPipelineResult));
        when(mockRightCam.getAllUnreadResults()).thenReturn(List.of(mockPipelineResult));

        // ...but the estimators cannot determine a pose.
        when(mockLeftEstimator.estimateCoprocMultiTagPose(mockPipelineResult)).thenReturn(Optional.empty());
        when(mockRightEstimator.estimateCoprocMultiTagPose(mockPipelineResult)).thenReturn(Optional.empty());

        // --- ACT ---
        List<EstimatedRobotPose> results = vision.getVisionUpdates();

        // --- ASSERT ---
        // We assert that the returned list is empty because no poses were added.
        assertTrue(results.isEmpty(), "The list of poses should be empty");

        verify(mockLeftCam).getAllUnreadResults();
        verify(mockRightCam).getAllUnreadResults();
        verify(mockLeftEstimator).estimateCoprocMultiTagPose(mockPipelineResult);
        verify(mockRightEstimator).estimateCoprocMultiTagPose(mockPipelineResult);
    }
}
