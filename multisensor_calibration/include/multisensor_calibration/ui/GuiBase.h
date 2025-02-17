// Copyright (c) 2024 - 2025 Fraunhofer IOSB and contributors
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Fraunhofer IOSB nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef MULTISENSORCALIBRATION_GUIBASE_H
#define MULTISENSORCALIBRATION_GUIBASE_H

// Std
#include <memory>
#include <string>

// ROS
#include <nodelet/loader.h>
#include <ros/ros.h>

// Qt
#include <QObject>
#include <QTimer>

namespace multisensor_calibration
{

/**
 * @ingroup ui
 * @brief Base class of all GUIs. This implements a common interface to all GUIs, e.g. init(),
 * setNodeletLoaderPtr().
 *
 */
class GuiBase : public QObject
{
    Q_OBJECT

    //--- METHOD DECLARATION ---//
  public:
    /**
     * @brief Default constructor.
     *
     * @param[in] iAppTitle Application title.
     * @param[in] iGuiSubNamespace Sub namespace of the gui.
     */
    GuiBase(const std::string& iAppTitle, const std::string& iGuiSubNamespace);

    /**
     * @brief Destructor
     */
    virtual ~GuiBase();

    /**
     * @brief Get name of gui node.
     */
    std::string getGuiNodeName() const;

    /**
     * @brief Constant reference to global node handle.
     */
    ros::NodeHandle& globalNodeHandle();

    /**
     * @brief Method to call the initialization routine. At the end of the routine the spin timer
     * is started to start the ROS spin loop.
     */
    virtual void init();

    /**
     * @brief Constant reference to private node handle.
     */
    ros::NodeHandle& privateNodeHandle();

    /**
     * @brief Method to set pointer to nodelet loader.
     *
     * @note This will not acquire ownership, thus the instance of the loader needs to be deleted
     * outside.
     */
    virtual void setNodeletLoaderPtr(std::shared_ptr<nodelet::Loader>& ipLoader);

    /**
     * @brief Method spinning ROS event loop once. This is connected to the spinTimer_.
     *
     */
    void spinOnce();

  signals:
    /**
     * @brief Signal emitted when ROS event loop is terminated.
     */
    void rosLoopTerminated();

    //--- MEMBER DECLARATION ---//

  protected:
    /// Application title
    std::string appTitle_;

    /// Name of the GUI node
    std::string guiNodeName_;

    /// Flag indicating if nodelet is initialized.
    bool isInitialized_;

    /// Global node handler.
    ros::NodeHandle nh_;

    /// Private node handler.
    ros::NodeHandle pnh_;

    /// Pointer to nodelet loader to load further nodelets
    std::shared_ptr<nodelet::Loader> pNodeletLoader_;

    /// QTimer object to trigger ros spins. This needs to be a QTimer, since the event loop runs in
    /// Qt.
    QTimer spinTimer_;
};

} // namespace multisensor_calibration

#endif // MULTISENSORCALIBRATION_GUIBASE_H