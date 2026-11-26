/* SPDX-License-Identifier: GPL-2.0-or-later */
#include <cmath>
#include <iostream>
#include <memory>

#include "cam_helper/cam_helper.h"
#include "test.h"

using namespace libcamera;
using namespace std::chrono_literals;
using RPiController::CamHelper;
using utils::Duration;

namespace libcamera {
LOG_DEFINE_CATEGORY(IPARPI)
}

class Imx585Test : public Test
{
	int run() override
	{
		std::unique_ptr<CamHelper> helper(CamHelper::create("imx585"));
		if (!helper || helper->hideFramesStartup() != 8 || helper->hideFramesModeSwitch() != 8 ||
		    helper->mistrustFramesStartup() != 8 || helper->mistrustFramesModeSwitch() != 8)
			return TestFail;

		for (uint32_t margin : { 8, 16 }) {
			for (uint32_t hmax : { 375, 550, 660, 1320 }) {
				if (!checkMode(*helper, hmax, margin))
					return TestFail;
			}
		}
		return TestPass;
	}

	bool checkMode(CamHelper &helper, uint32_t base, uint32_t margin)
	{
		constexpr uint64_t clock = 74250000;
		CameraMode mode{};
		mode.width = 3840;
		mode.height = margin == 16 ? 2200 : 2160;
		mode.pixelRate = mode.width * clock / base;
		mode.minLineLength = mode.width * (1.0s / mode.pixelRate);
		const uint32_t maxPixels = (65536ULL * mode.width + base - 1) / base - 1;
		mode.maxLineLength = maxPixels * (1.0s / mode.pixelRate);
		mode.minFrameLength = margin == 16 ? 4500 : 2250;
		mode.maxFrameLength = 0xffffe;
		helper.setFrameIntegrationDiff(margin);

		for (bool ispLimited : { false, true }) {
			if (ispLimited)
				mode.minLineLength = (base + 4.2) * (1.0s / clock);
			helper.setCameraMode(mode);
			const Duration min = mode.minLineLength * mode.minFrameLength;
			const Duration max = mode.maxLineLength * mode.maxFrameLength;
			for (Duration period : { Duration(0s), Duration(33333us), Duration(30s), max }) {
				for (Duration request : std::initializer_list<Duration>{ 0s, 3 * mode.minLineLength, 17ms,
							  Duration(20s), Duration::max() }) {
					Duration exposure = request;
					const auto [vb, hb] = helper.getBlanking(exposure, period ? period : min,
										 period ? period : max);
					const uint32_t h = uint64_t(base) * (mode.width + hb) / mode.width;
					const uint32_t v = mode.height + vb;
					const Duration line = helper.hblankToLineLength(hb);
					const uint32_t lines = helper.exposureLines(exposure, line);
					if (h < base + (ispLimited ? 5 : 0) || h > 65535 ||
					    v < mode.minFrameLength || v > mode.maxFrameLength || v % 2 ||
					    lines < 2 || lines > v - margin || lines % 2 ||
					    std::abs(line * clock / 1.0s - h) > 1e-8 ||
					    exposure != helper.exposure(lines, line)) {
						std::cerr << "Unrepresentable timing: HMAX " << h << " VMAX " << v
							  << " exposure lines " << lines << std::endl;
						return false;
					}
					if ((period == max || (!period && request == Duration::max())) &&
					    (h != 65535 || v != mode.maxFrameLength))
						return false;
					if (request == Duration::max() && lines != v - margin)
						return false;
					if (period == 33333us && base == 660 && margin == 8 &&
					    uint64_t(h) * v != clock / 30)
						return false;
				}
			}
		}
		return true;
	}
};

TEST_REGISTER(Imx585Test)
