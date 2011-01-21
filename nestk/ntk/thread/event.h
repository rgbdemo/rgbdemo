/**
 * This file is part of the nestk library.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Nicolas Burrus <nicolas.burrus@uc3m.es>, (C) 2010
 */

#ifndef NTK_THREAD_EVENT_H
#define NTK_THREAD_EVENT_H

#include <ntk/core.h>
#include <ntk/thread/utils.h>
#include <ntk/utils/debug.h>

#include <QWriteLocker>
#include <QWaitCondition>
#include <QMutex>

#include <list>

namespace ntk
{

  class EventListener
  {
  public:
    virtual void newEvent(void* sender = 0) = 0;
  };

  class SyncEventListener : public EventListener
  {
  public:
    SyncEventListener() : m_enabled(true), m_unprocessed_event(false)
    {}

    void setEnabled(bool enabled) { m_enabled = enabled; }
    bool enabled() const { return m_enabled; }

    virtual void newEvent(void* sender = 0);
    void waitForNewEvent(int timeout_msecs = 60000);

  private:
    bool m_enabled;
    bool m_unprocessed_event;
    mutable QMutex m_lock;
    QWaitCondition m_condition;
  };

class AsyncEventListener : public QObject, public EventListener
{
  Q_OBJECT

public:
  AsyncEventListener() : m_ready(true)
  {}

  virtual void newEvent(void* sender = 0);
  virtual void handleAsyncEvent() = 0;
  virtual void customEvent(QEvent* event);

private:
  bool m_ready; // FIXME: need for a lock here?
};

class EventBroadcaster
{
public:
  void addEventListener(EventListener* updater);
  void removeAllEventListeners();
  void broadcastEvent(void* sender = 0);

private:
  std::vector<EventListener*> m_listeners;
};

} // ntk

#endif // NTK_THREAD_EVENT_H
