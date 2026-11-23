/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * black level control algorithm interface
 */
#pragma once

#include <optional>

#include "algorithm.h"

namespace RPiController {

class BlackLevelAlgorithm : public Algorithm
{
public:
	BlackLevelAlgorithm(Controller *controller)
		: Algorithm(controller) {}
	/* A black level algorithm must provide the following: */
	virtual void initialValues(uint16_t &blackLevelR, uint16_t &blackLevelG,
				   uint16_t &blackLevelB) = 0;
	void setSensorBlackLevel(std::optional<uint16_t> level) { sensorBlackLevel_ = level; }

protected:
	std::optional<uint16_t> sensorBlackLevel_;
};

} /* namespace RPiController */
