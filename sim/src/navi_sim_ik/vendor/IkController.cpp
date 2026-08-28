//
// Created by alex on 31.08.22.
//

#include "IkController.h"
#include <iostream>

void IkController::update() {

    using namespace std::chrono_literals;
    using clock = std::chrono::system_clock;

    auto workStart = clock::now();

    while (!m_stopUpdater) {
        auto workEnd = clock::now();
        auto sleepStart = clock::now();
        std::cout << "work took " << (workEnd - workStart).count() / 1000000.f << "\n";
        if ((workEnd - workStart).count() / 1000000.f > 60) {
            std::cout << "Deadline VIOLATED!\n";
        }

        std::this_thread::sleep_for(0.06s - (workEnd - workStart));

        auto diff = clock::now() - sleepStart;
        std::cout << "expected: " << (0.06s - (workEnd - workStart)).count() / 1000000.f << "\n"
            << "actual: " << diff.count() / 1000000.f << "\n";

        workStart = clock::now();
        {
            // overrun flag not required, since only one thread accesses this function
            m_updateVars(m_in);
            m_model.setExternalInputs(&m_in);

            // step the model
            m_model.step();

            m_out = m_model.getExternalOutputs();
            m_outputFunc(m_out, m_in);
        }
    }
}
