
#include "ai.h"

//signal_from_buffer();

static size_t frame_surplus;    // Edge inmpulse


void result_ready_cb(int err)
{
	if (err) {
		printk("Result ready callback returned error (err: %d)\n", err);
		return;
	}

	const char *label;
	float value;
	float anomaly;

	printk("\nClassification results\n");
	printk("======================\n");

	while (true) {
		err = ei_wrapper_get_next_classification_result(&label, &value, NULL);

		if (err) {
			if (err == -ENOENT) {
				err = 0;
			}
			break;
		}

		printk("Value: %.2f\tLabel: %s\n", value, label);
	}

	if (err) {
		printk("Cannot get classification results (err: %d)", err);
	} else {
		if (ei_wrapper_classifier_has_anomaly()) {
			err = ei_wrapper_get_anomaly(&anomaly);
			if (err) {
				printk("Cannot get anomaly (err: %d)\n", err);
			} else {
				printk("Anomaly: %.2f\n", anomaly);
			}
		}
	}

	if (frame_surplus > 0) {
		err = ei_wrapper_start_prediction(0, 1);
		if (err) {
			printk("Cannot restart prediction (err: %d)\n", err);
		} else {
			printk("Prediction restarted...\n");
		}

		frame_surplus--;
	}
}