/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021, Raspberry Pi Ltd
 *
 * cam_helper_Imx585.cpp - camera information for Imx585 sensor
 */

#include <assert.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include "cam_helper.h"
#include "math.h"
using namespace RPiController;
using namespace libcamera;
using libcamera::utils::Duration;
using namespace std::literals::chrono_literals;

class CamHelperImx585 : public CamHelper
{
public:
	CamHelperImx585();
	uint32_t gainCode(double gain) const override;
	double gain(uint32_t gainCode) const override;
	uint32_t exposureLines(Duration exposure, Duration lineLength) const override;
	Duration hblankToLineLength(uint32_t hblank) const override;
	unsigned int hideFramesStartup() const override;
	unsigned int hideFramesModeSwitch() const override;
	unsigned int mistrustFramesStartup() const override;
	unsigned int mistrustFramesModeSwitch() const override;
	std::pair<uint32_t, uint32_t>
	getBlanking(Duration &exposure, Duration minFrameDuration,
		    Duration maxFrameDuration) const override;

private:
	static constexpr uint64_t sensorClock = 74250000;
	uint32_t nativeHmax() const;
	uint32_t hmaxToHblank(uint32_t hmax) const;
	uint32_t hblankToHmax(uint32_t hblank) const;
};

/*
 * Imx585 doesn't output metadata, so we have to use the "unicam parser" which
 * works by counting frames.
 */

CamHelperImx585::CamHelperImx585()
	: CamHelper({}, 16)
{
}


uint32_t CamHelperImx585::gainCode(double gain) const
{
	int code = 66.6667 * log10(gain);
	return std::max(0, std::min(code, 0xf0));
}

double CamHelperImx585::gain(uint32_t gainCode) const
{
	return pow(10, 0.015 * gainCode);
}

uint32_t CamHelperImx585::exposureLines(Duration exposure, Duration lineLength) const
{
	/* Both VMAX and SHR0 are even. Limit before converting an unbounded duration. */
	const double pairs = std::clamp(exposure / (2 * lineLength), 1.0,
					double(mode_.maxFrameLength / 2));
	return 2 * std::lround(pairs);
}

/* The driver reports floor(width * sensorClock / native HMAX) as pixel rate. */
uint32_t CamHelperImx585::nativeHmax() const
{
	return (mode_.width * sensorClock + mode_.pixelRate / 2) / mode_.pixelRate;
}

uint32_t CamHelperImx585::hmaxToHblank(uint32_t hmax) const
{
	const uint32_t base = nativeHmax();
	return (uint64_t(hmax) * mode_.width + base - 1) / base - mode_.width;
}

uint32_t CamHelperImx585::hblankToHmax(uint32_t hblank) const
{
	return uint64_t(nativeHmax()) * (mode_.width + hblank) / mode_.width;
}

Duration CamHelperImx585::hblankToLineLength(uint32_t hblank) const
{
	return hblankToHmax(hblank) * (1.0s / sensorClock);
}

unsigned int CamHelperImx585::hideFramesStartup() const
{
	/* Sony IMX585 Software Reference Manual, Rev. 0.3, p. 7. */
	return 8;
}

unsigned int CamHelperImx585::hideFramesModeSwitch() const
{
	/* Each stream restart releases the sensor from standby again. */
	return hideFramesStartup();
}

unsigned int CamHelperImx585::mistrustFramesStartup() const
{
	return hideFramesStartup();
}

unsigned int CamHelperImx585::mistrustFramesModeSwitch() const
{
	return hideFramesModeSwitch();
}

/* The frame period is the product of even VMAX and integer HMAX in sensor ticks. */
std::pair<uint32_t, uint32_t>
CamHelperImx585::getBlanking(Duration &exposure, Duration minFrameDuration,
			     Duration maxFrameDuration) const
{
	const auto toTicks = [](Duration duration) -> double {
		return duration / (1.0s / sensorClock);
	};
	const uint32_t maxPixels = std::llround(mode_.maxLineLength * mode_.pixelRate / 1.0s);
	const uint32_t maxH = hblankToHmax(maxPixels - mode_.width);
	const uint32_t minH = mode_.minLineLength <= Duration(mode_.width * (1.0s / mode_.pixelRate))
			      ? nativeHmax()
			      : std::min<uint32_t>(std::ceil(toTicks(mode_.minLineLength)), maxH);
	const uint32_t minV = utils::alignUp(mode_.minFrameLength, 2);
	const uint32_t maxV = utils::alignDown(mode_.maxFrameLength, 2);
	const double shortest = double(minH) * minV;
	const double longest = double(maxH) * maxV;
	const double minTicks = std::clamp(toTicks(minFrameDuration), shortest, longest);
	const double maxTicks = std::clamp(toTicks(maxFrameDuration), minTicks, longest);
	const double exposureTicks = std::clamp(toTicks(exposure), 0.0, maxTicks);

	/* Stretch the line only when the maximum frame length requires it. */
	const double requiredH = std::clamp(exposureTicks / (maxV - frameIntegrationDiff_),
					    minTicks / maxV, maxTicks / maxV);
	uint32_t bestH = std::clamp<uint32_t>(std::ceil(requiredH), minH, maxH);
	const double wanted = std::clamp(exposureTicks + frameIntegrationDiff_ * double(bestH),
					 minTicks, maxTicks);
	auto nearestEven = [&](double ticks, uint32_t hmax) -> uint32_t {
		return std::clamp(2.0 * std::round(ticks / (2 * hmax)), double(minV), double(maxV));
	};

	uint32_t bestV = nearestEven(wanted, bestH);
	const bool fixed = minFrameDuration == maxFrameDuration;
	if (!fixed) {
		/* Stay inside the requested interval when it contains an even frame length. */
		const double lower = std::max<double>(minV, 2 * std::ceil(minTicks / (2 * bestH)));
		const double upper = std::min<double>(maxV, 2 * std::floor(maxTicks / (2 * bestH)));
		if (lower <= upper)
			bestV = std::clamp<double>(bestV, lower, upper);
	} else {
		int64_t target = std::llround(wanted);

		/* Recover nominal integer rates from the API's whole-microsecond period. */
		const long rate = std::lround(double(sensorClock) / target);
		if (rate > 0 && std::abs(1.0e6 / rate - minFrameDuration.get<std::micro>()) < 1.0 &&
		    sensorClock % (2 * rate) == 0)
			target = sensorClock / rate;

		/* Prefer an exact period, then the shortest line within half a microsecond. */
		const uint32_t firstH = bestH;
		const uint32_t lastH = std::min(maxH, firstH + minH / 2);
		const int64_t halfUs = std::llround(sensorClock / 2.0e6);
		int64_t bestError = std::numeric_limits<int64_t>::max();
		for (uint32_t h = firstH; h <= lastH; ++h) {
			const uint32_t v = nearestEven(target, h);
			const int64_t error = std::abs(int64_t(v) * h - target);
			if (!error || (bestError > halfUs && error < bestError)) {
				bestH = h;
				bestV = v;
				bestError = error;
			}
			if (!bestError)
				break;
		}
	}

	const uint32_t hblank = hmaxToHblank(bestH);
	const Duration lineLength = hblankToLineLength(hblank);
	const uint32_t lines = std::min(exposureLines(exposure, lineLength), bestV - frameIntegrationDiff_);
	exposure = this->exposure(lines, lineLength);
	return { bestV - mode_.height, hblank };
}


static CamHelper *create()
{
	return new CamHelperImx585();
}

static RegisterCamHelper reg("imx585", &create);
