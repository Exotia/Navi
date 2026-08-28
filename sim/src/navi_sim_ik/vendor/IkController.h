//
// Created by alex on 31.08.22.
//

#ifndef BEMACONTROLLER_IKCONTROLLER_H
#define BEMACONTROLLER_IKCONTROLLER_H

#include <atomic>
#include <functional>
#include <mutex>
#include <sstream>
#include <thread>
#include <utility>

#include "kinematics.h"

class IkController {
public:
    using Input = kinematics::ExternalInputs;
    using Output = kinematics::ExternalOutputs;

    explicit IkController(std::function<void(Input&)>  updateVarsFunc, std::function<void(const Output&, const Input&)> outputFunc) :
            m_updateVars(std::move(updateVarsFunc)), m_outputFunc(std::move(outputFunc)),
            m_stopUpdater(false), m_stopped(true) {
        // initialize the models input parameters
        m_model.initialize();

        // set U_TS to constant value of 0.06[s]
        m_in.TS = 0.06;
    }

    virtual ~IkController() {
        if (!m_stopped) stop();
    }

    void start() {
        if (m_stopped.exchange(false)) {
            m_stopUpdater = false;
            m_updater = std::thread([this]() { update(); });
        }
    }

    void stop() {
        if (!m_stopped.exchange(true)) {
            m_stopUpdater = true;
            m_updater.join();
        }
    }

    auto serialize() const {
        std::stringstream ss;
        ss << "\"driveAlgorithm\":{";

        ss << "\"input_ICR\":[" << m_out.input_ICR[0] << "," << m_out.input_ICR[1] << "],"
           << "\"controller_ICR\":[" << m_out.controller_ICR[0] << "," << m_out.controller_ICR[1] << "],"
           << "\"feasable_ICR\":[" << m_out.feasable_ICR[0] << "," << m_out.feasable_ICR[1] << "],"
           << "\"current_ICR\":[" << m_out.current_ICR[0] << "," << m_out.current_ICR[1] << "]"
        << "}";
        return  ss.str();
    }

private:
    kinematics m_model{};
    Input m_in{};
    Output m_out{};

    std::thread m_updater;
    std::atomic_bool m_stopUpdater, m_stopped;
    std::function<void(Input&)> m_updateVars;
    std::function<void(Output&, const Input&)> m_outputFunc;

    void update();
};

#endif //BEMACONTROLLER_IKCONTROLLER_H
