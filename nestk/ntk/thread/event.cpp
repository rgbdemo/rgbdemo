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

#include "event.h"

#include <QCoreApplication>

namespace ntk
{

  void SyncEventListener :: newEvent(void* sender)
  {
    if (!m_enabled) return;
    m_lock.lock();
    m_unprocessed_event = true;
    m_condition.wakeAll();
    m_lock.unlock();
  }

  void SyncEventListener :: waitForNewEvent(int timeout_msecs)
  {
    if (!m_enabled) return;
    m_lock.lock();
    if (!m_unprocessed_event)
    {
      m_condition.wait(&m_lock, timeout_msecs);
    }
    m_unprocessed_event = false;
    m_lock.unlock();
  }

void AsyncEventListener :: newEvent(void* sender)
{
  {
    if (!m_ready)
      return;

    m_ready = false;
  }
  QCoreApplication::postEvent(this, new QEvent(QEvent::User));
}

void AsyncEventListener :: customEvent(QEvent* event)
{
  if (event->type() != QEvent::User)
    return QObject::customEvent(event);

  event->accept();
  m_ready = true;
  handleAsyncEvent();
}

void EventBroadcaster :: addEventListener(EventListener* updater)
{
  m_listeners.push_back(updater);
}

void EventBroadcaster :: removeAllEventListeners()
{
  m_listeners.clear();
}

void EventBroadcaster :: broadcastEvent(void* sender)
{
  foreach_idx(i, m_listeners)
    m_listeners[i]->newEvent(sender);
}

} // ntk
