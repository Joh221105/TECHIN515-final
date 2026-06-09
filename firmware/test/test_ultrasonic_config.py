import unittest
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


class UltrasonicConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.main_cpp = (ROOT / "src" / "main.cpp").read_text()
        cls.platformio = (ROOT / "platformio.ini").read_text()

    def test_audio_library_sample_rate_is_72khz(self):
        self.assertIn("-D AUDIO_SAMPLE_RATE_EXACT=72000.0f", self.platformio)

    def test_firmware_sample_rate_uses_audio_library_rate(self):
        self.assertRegex(
            self.main_cpp,
            r"#define\s+SAMPLE_RATE\s+\(\(float\)AUDIO_SAMPLE_RATE_EXACT\)",
        )

    def test_profiles_target_ultrasonic_leak_band(self):
        profile_lines = [
            line.strip()
            for line in self.main_cpp.splitlines()
            if line.strip().startswith('{"')
        ]
        ultrasonic_profiles = [
            line for line in profile_lines
            if "20000.0f" in line and "36000.0f" in line
        ]
        self.assertGreaterEqual(len(ultrasonic_profiles), 2)

    def test_status_output_exposes_detection_components(self):
        for field in (
            "spectral_snr_db",
            "level_rise_db",
            "peak_snr_db",
            "pinpoint_metric_db",
            "audible_metric_db",
            "confirm_count",
        ):
            self.assertIn(field, self.main_cpp)

    def test_startup_prints_rate_and_pdm_clock(self):
        self.assertIn("Nyquist", self.main_cpp)
        self.assertIn("PDM clock", self.main_cpp)

    def test_status_updates_are_more_responsive(self):
        self.assertIn("#define PRINT_INTERVAL_MS      100", self.main_cpp)

    def test_profiles_have_lower_latency_confirmation_counts(self):
        self.assertIn('"ULTRASONIC_BALANCED",      22000.0f, 34000.0f, 12000.0f, 19000.0f, 6.5f,  8', self.main_cpp)
        self.assertIn('"ULTRASONIC_STABLE",        24000.0f, 34000.0f, 12000.0f, 19000.0f, 8.0f, 12', self.main_cpp)
        self.assertIn('"ULTRASONIC_SENSITIVE",     20000.0f, 36000.0f, 10000.0f, 18000.0f, 4.5f, 10', self.main_cpp)

    def test_led_has_gradient_smoothing_and_strong_signal_blink(self):
        for token in (
            "led_intensity_smoothed",
            "LED_ATTACK_ALPHA",
            "LED_RELEASE_ALPHA",
            "STRONG_SIGNAL_BLINK_MS",
            "strong_signal",
            "pixels.Color(255, 0, 0)",
            "pixels.Color(0, 0, 0)",
        ):
            self.assertIn(token, self.main_cpp)

    def test_strong_signal_blink_is_rapid_and_refreshed_outside_fft_gate(self):
        self.assertIn("#define STRONG_SIGNAL_BLINK_MS   60", self.main_cpp)
        self.assertIn("#define STRONG_SIGNAL_HOLD_MS   500", self.main_cpp)
        self.assertIn("last_strong_signal_ms", self.main_cpp)
        self.assertIn("updateLED(new_fft)", self.main_cpp)
        self.assertNotIn("if (new_fft) updateLED();", self.main_cpp)

    def test_pinpoint_profile_averages_narrow_higher_band(self):
        self.assertIn(
            '"PINPOINT_HIGH_BAND",      28000.0f, 36000.0f, 18000.0f, 26000.0f, 1.0f,  4, 0.018f, 0.002f, 8.0f, true',
            self.main_cpp,
        )

    def test_pinpoint_detection_uses_top_bin_average(self):
        for token in (
            "#define PINPOINT_TOP_BINS         3",
            "topBinAveragePower",
            "peak_snr_db",
            "pinpoint_metric_db = fmaxf(0.0f, fmaxf(peak_snr_db, spectral_snr_db));",
            "PEAKSNR=%.1f",
        ):
            self.assertIn(token, self.main_cpp)

    def test_pinpoint_detection_uses_score_latch_for_intermittent_bursts(self):
        for token in (
            "#define PINPOINT_PROFILE_ID       6",
            "#define PINPOINT_SCORE_HIT        3",
            "#define PINPOINT_SCORE_NEAR       2",
            "#define PINPOINT_SCORE_WEAK       1",
            "#define PINPOINT_SCORE_DECAY      1",
            "#define PINPOINT_SCORE_CONFIRM    4",
            "#define PINPOINT_SCORE_RELEASE    1",
            "#define PINPOINT_SCORE_MAX       10",
            "#define PINPOINT_NEAR_MARGIN_DB 0.4f",
            "#define PINPOINT_WEAK_SNR_DB    0.2f",
            "pinpoint_score",
            "updatePinpointScore",
            "activeProfileId == PINPOINT_PROFILE_ID",
            "snr_db >= activeProfile->snrThresholdDb - PINPOINT_NEAR_MARGIN_DB",
            "snr_db >= PINPOINT_WEAK_SNR_DB",
        ):
            self.assertIn(token, self.main_cpp)

    def test_pinpoint_detection_holds_through_brief_dropouts(self):
        for token in (
            "if (pinpoint_score >= PINPOINT_SCORE_CONFIRM) acoustic_leak = true;",
            "else if (pinpoint_score <= PINPOINT_SCORE_RELEASE) acoustic_leak = false;",
            "if (pinpoint_score > PINPOINT_SCORE_MAX) pinpoint_score = PINPOINT_SCORE_MAX;",
        ):
            self.assertIn(token, self.main_cpp)
        self.assertNotIn("acoustic_leak = pinpoint_score >= PINPOINT_SCORE_CONFIRM;", self.main_cpp)

    def test_pinpoint_profile_ignores_raw_level_rise_for_detection_score(self):
        for token in (
            "combined_snr_db = fmaxf(0.0f, fmaxf(spectral_snr_db, level_rise_db));",
            "combined_snr_db = fmaxf(combined_snr_db, peak_snr_db);",
            "snr_db = isPinpointProfile() ? pinpoint_metric_db : combined_snr_db;",
            "PINPOINT=%.1f",
        ):
            self.assertIn(token, self.main_cpp)
        direct_rise_assignment = re.compile(
            r"^\s*snr_db\s*=\s*fmaxf\(0\.0f,\s*fmaxf\(spectral_snr_db,\s*level_rise_db\)\);",
            re.MULTILINE,
        )
        self.assertIsNone(direct_rise_assignment.search(self.main_cpp))

    def test_pinpoint_profile_has_clean_audible_fallback_for_loud_hisses(self):
        for token in (
            "#define AUDIBLE_FALLBACK_LOW_HZ     6000.0f",
            "#define AUDIBLE_FALLBACK_HIGH_HZ   20000.0f",
            "#define AUDIBLE_FALLBACK_NOISE_LOW_HZ 1000.0f",
            "#define AUDIBLE_FALLBACK_NOISE_HIGH_HZ 5000.0f",
            "#define AUDIBLE_FALLBACK_GATE_DB    6.0f",
            "audible_metric_db = fmaxf(0.0f, audible_snr_db - AUDIBLE_FALLBACK_GATE_DB);",
            "pinpoint_metric_db = fmaxf(pinpoint_metric_db, audible_metric_db);",
            "AUDIBLE=%.1f",
        ):
            self.assertIn(token, self.main_cpp)

    def test_serial_output_exposes_raw_and_ambient_signal_levels(self):
        for token in (
            "SIGDB=%.1f",
            "AMBDB=%.1f",
            "signal_level_db",
            "ambient_level_db",
        ):
            self.assertIn(token, self.main_cpp)

    def test_serial_peak_output_is_khz(self):
        self.assertIn("PEAK=%.1fkHz", self.main_cpp)
        self.assertIn("peak_freq / 1000.0f", self.main_cpp)
        self.assertNotIn("PEAK=%.0fHz", self.main_cpp)


if __name__ == "__main__":
    unittest.main()
