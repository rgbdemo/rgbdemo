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

#ifndef NTK_utils_xml_serializable_H
# define NTK_utils_xml_serializable_H

# include <ntk/core.h>
# include <ntk/utils/qt_utils.h>
# include <ntk/utils/serializable.h>
# include <ntk/utils/xml_parser.h>

# include <iostream>

# include <QFileInfo>
# include <QString>

using ntk::XMLNode;

namespace ntk
{

  class XmlSerializable
  {
    public:
      virtual ~XmlSerializable() {}

    public:
      virtual void fillXmlElement(XMLNode& element) const = 0;

      virtual void loadFromXmlElement(const XMLNode& element)
      { ntk_assert(0, "Implement this in children."); }

      virtual void loadFromXmlElement(const XMLNode& element, XmlContext* context)
      { ntk_assert(context == 0, "Context ignored."); loadFromXmlElement(element); }

      virtual void saveAsXml(const QFileInfo& file, const char* element_tag = "root") const;
      virtual QString getAsXml(const char* element_tag = "root") const;

      virtual void loadFromXml(const QFileInfo& file, const char* element_tag = "root", XmlContext* context=0);
      virtual void loadFromXml(const QString& s, const char* element_tag = "root", XmlContext* context=0);
      virtual void loadFromXml(XMLNode& doc, const char* element_tag = "root", XmlContext* context=0);

    protected:
      XMLNode addXmlChild(XMLNode& element, const char* tag, const XmlSerializable& object) const;
      
      // faster.
      XMLNode addXmlChildAtBeginning(XMLNode& element, const char* tag, const XmlSerializable& object) const;
      
      void loadFromXmlChild(const XMLNode& element, const char* tag,
                            XmlSerializable& object, XmlContext* context = 0);

      template <typename Type>
      void setXmlAttribute(XMLNode& element, const char* attr_name, const Type& value) const
      {
        QString s;
        QTextStream stream(&s); stream << value;
        element.addAttribute(attr_name, s.toUtf8());
      }

      void setXmlAttribute(XMLNode& element, const char* attr_name, const QString& value) const
      {
        element.addAttribute(attr_name, (const char*)value.toUtf8());
      }

      void setXmlAttribute(XMLNode& element, const char* attr_name, const char* value) const
      {
        element.addAttribute(attr_name, value);
      }

      void setXmlAttribute(XMLNode& element, const char* attr_name, const std::string& value) const
      {
        element.addAttribute(attr_name, value.c_str());
      }

      template <typename Type>
      void loadFromXmlAttribute(const XMLNode& element, const char* attr_name, Type& value)
      {
        if (!element.getAttribute(attr_name))
          ntk_throw_exception("Missing attribute: " + attr_name);
        QString s = element.getAttribute(attr_name);
        QTextStream stream(&s);
        stream >> value;
      }

      void loadFromXmlAttribute(const XMLNode& element, const char* attr_name, QString& value)
      {
        const char* cstr;
        if (!(cstr = element.getAttribute(attr_name)))
          ntk_throw_exception("Missing attribute: " + attr_name);
        value = cstr;
      }

      void loadFromXmlAttribute(const XMLNode& element, const char* attr_name, std::string& value)
      {
        const char* cstr;
        if (!(cstr = element.getAttribute(attr_name)))
          ntk_throw_exception("Missing attribute: " + attr_name);
        value = cstr;
      }

      template <typename Type>
      void setXmlRawData(XMLNode& element, const Type& value) const
      {
        QByteArray data;
        { // block ensures data is flushed.
          QDataStream stream(&data, QIODevice::WriteOnly | QIODevice::Unbuffered);
          stream << value;
        }
        element.addText(data.toBase64().constData(), 0);
      }

      template <typename Type>
      void loadFromXmlRawData(const XMLNode& element, Type& value)
      {
        QByteArray data = QByteArray::fromBase64(element.getText());
        QDataStream stream(data);
        stream >> value;
        ntk_throw_exception_if(stream.status() != QDataStream::Ok, "Could not load binary data.");
      }

      void setXmlRawTextData(XMLNode& element, const QString& value) const
      {
        // FIXME: escape xml symbols.
        element.addText(value.toUtf8(), 0);
      }

      void loadFromXmlRawTextData(const XMLNode& element, QString& value)
      {
        // FIXME: escape xml symbols.
        value = element.getText();
      }

      template <typename Type>
      void addXmlRawTextDataChild(XMLNode& element, const char* tag, const Type& value) const
      {
        QString s;
        QTextStream stream(&s); stream << value;

        XMLNode data_element = element.addChild(tag, 0);
        setXmlRawTextData(data_element, s);
      }

      template <typename Type>
      void loadFromXmlRawTextDataChild(const XMLNode& element, const char* tag, Type& value)
      {
        XMLNode data_element = element.getChildNode(tag);
        ntk_throw_exception_if(data_element.isEmpty(), "Could not find raw data for tag " + tag);

        QString s;
        loadFromXmlRawTextData(data_element, s);
        QTextStream stream(&s);
        stream >> value;
      }

      template <typename Type>
      void addXmlRawDataChild(XMLNode& element, const char* tag, const Type& value) const
      {
        XMLNode data_element = element.addChild(tag);
        setXmlRawData(data_element, value);
      }

      template <typename Type>
      void loadFromXmlRawDataChild(const XMLNode& element, const char* tag, Type& value)
      {
        XMLNode data_element = element.getChildNode(tag);
        ntk_throw_exception_if(data_element.isEmpty(), "Could not find raw data for tag " + tag);
        loadFromXmlRawData(data_element, value);
      }
  };
  ntk_ptr_typedefs(XmlSerializable);

  template <class T>
  class VectorPtrSerializer : public XmlSerializable
  {
      public:
        typedef std::vector<T*> value_type;
        typedef VectorPtrSerializer<T> this_type;

      public:
        static const XmlSerializableConstPtr get(const value_type& value, XmlContext* context = 0)
        { return XmlSerializableConstPtr(new this_type(value, context)); }

        static const XmlSerializablePtr get(value_type& value, XmlContext* context = 0)
        { return XmlSerializablePtr(new this_type(value, context)); }

        virtual void fillXmlElement(XMLNode& element) const
        {
          setXmlAttribute(element, "size", (int)value.size());
          foreach_idx(i, value)
          {
            XMLNode child = element.addChild("value");
            setXmlAttribute(child, "index", (int)i);
            value[i]->fillXmlElement(child);
          }
        }

        virtual void loadFromXmlElement(const XMLNode& element)
        {
          int size = 0;
          loadFromXmlAttribute(element, "size", size);
          value.resize(size, 0);

          for (int i = 0; i < element.nChildNode(); ++i)
          {
            XMLNode e = element.getChildNode(i);
            if (e.getNameAsString() != "value") continue;

            T* obj = new T();
            if (context)
              obj->loadFromXmlElement(e, context);
            else
              obj->loadFromXmlElement(e);
            int index; loadFromXmlAttribute(e, "index", index);
            value[index] = obj;
          }
        }

      private:
        VectorPtrSerializer(const value_type& value, XmlContext* context = 0)
          : value(const_cast<value_type&>(value)), context(context)
        {}

      private:
        value_type& value;
        XmlContext* context;
    };

  template <class T>
  const XmlSerializableConstPtr getVectorPtrSerializer(const std::vector<T*>& value, XmlContext* context = 0)
  { return VectorPtrSerializer<T>::get(value, context); }

  template <class T>
  static const XmlSerializablePtr getVectorPtrSerializer(std::vector<T*>& value, XmlContext* context = 0)
  { return VectorPtrSerializer<T>::get(value, context); }

} // end of ntk

#endif // ndef NTK_utils_xml_serializable_H
