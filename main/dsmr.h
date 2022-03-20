#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <cjson.h>
#include "lib/misc/datetime.h"

namespace DSMR
{
		 
	const char* capture =
	"/KMP5 ZABF001587315111 (KMP lijkt de identificatie te zijn van Kamstrup)                                                     \r\n"
	"0-0:96.1.1(205C4D246333034353537383234323121) (serienummer meter in hexadecimale ascii code)                                 \r\n"
	"1-0:1.8.1(00185.000*kWh) (Totaal verbruik tarief 1 (nacht))                                                                  \r\n"
	"1-0:1.8.2(00084.000*kWh) (Totaal verbruik tarief 2 (dag))                                                                    \r\n"
	"1-0:2.8.1(00013.000*kWh) (Totaal geleverd tarief 1 (nacht))                                                                  \r\n"
	"1-0:2.8.2(00019.000*kWh) (Totaal geleverd tarief 2 (dag))                                                                    \r\n"
	"0-0:96.14.0(0001) (Actuele tarief (1))                                                                                       \r\n"
	"1-0:1.7.0(0000.98*kW) (huidig verbruik)                                                                                      \r\n"
	"1-0:2.7.0(0000.00*kW) (huidige teruglevering)                                                                                \r\n"
	"0-0:17.0.0(999*A) (maximum stroom per fase                                                                                   \r\n"
	"0-0:96.3.10(1) (stand van de schakelaar)                                                                                     \r\n"
	"0-0:96.13.1() (bericht numeriek)                                                                                             \r\n"
	"0-0:96.13.0() (bericht tekst)                                                                                                \r\n"
	"0-1:24.1.0(3) (andere apparaten op de M-Bus)                                                                                 \r\n"
	"0-1:96.1.0(3238313031453631373038389930337131) (identificatie van de gasmeter)                                               \r\n"
	"0-1:24.3.0(120517020000)(08)(60)(1)(0-1:24.2.1)(m3) (tijd van de laatste gas meting (120517020000 = 17 mei 2012 2uur))       \r\n"
	"(00124.477) (Verbruikte hoeveelheid gas)                                                                                     \r\n"
	"0-1:24.4.0(1) (stand gasklep?)                                                                                               \r\n"
	"! (Afsluiter)                                                                                                                \r\n";



	struct Measurement
	{
		DateTime TimeStamp;
		float TotalConsumption1;
		float TotalConsumption2;
		float TotalProduction1;
		float TotalProduction2;
		float ActualConsumption;
		float ActualProduction;
		float TotalGasConsumption;
		uint32_t ActualTarrif;
		
		std::string ToString()
		{
			char buf[1024];
			int pos = 0;
			pos += sprintf(&buf[pos], "Time = %s\n", TimeStamp.ToString().c_str());
			pos += sprintf(&buf[pos], "TotalConsumption1 = %f\n",	TotalConsumption1);
			pos += sprintf(&buf[pos], "TotalConsumption2 = %f\n",	TotalConsumption2);
			pos += sprintf(&buf[pos], "TotalProduction1 = %f\n",	TotalProduction1);
			pos += sprintf(&buf[pos], "TotalProduction2 = %f\n",	TotalProduction2);
			pos += sprintf(&buf[pos], "ActualConsumption = %f\n",	ActualConsumption);
			pos += sprintf(&buf[pos], "ActualProduction = %f\n",	ActualProduction);
			pos += sprintf(&buf[pos], "TotalGasConsumption = %f\n", TotalGasConsumption);
			pos += sprintf(&buf[pos], "ActualTarrif = %d\n",		ActualTarrif);
			return buf;
		}
		
		
		cJSON* ToJSON()
		{
			cJSON* obj = cJSON_CreateObject();
			cJSON_AddStringToObject(obj, "TimeStamp", TimeStamp.ToString().c_str());
			cJSON_AddNumberToObject(obj, "TotalConsumption1", TotalConsumption1);
			cJSON_AddNumberToObject(obj, "TotalConsumption2", TotalConsumption2);
			cJSON_AddNumberToObject(obj, "TotalProduction1", TotalProduction1);
			cJSON_AddNumberToObject(obj, "TotalProduction2", TotalProduction2);
			cJSON_AddNumberToObject(obj, "ActualConsumption", ActualConsumption);
			cJSON_AddNumberToObject(obj, "ActualProduction", ActualProduction);
			cJSON_AddNumberToObject(obj, "TotalGasConsumption", TotalGasConsumption);
			cJSON_AddNumberToObject(obj, "ActualTarrif", ActualTarrif);
			return obj;
		}
		
	};


	struct Parser
	{

		static int ParseFloat(std::string& raw, int pos, float* val)
		{
			size_t start = raw.find('(', pos) + 1;
			if (start >= 0)
			{
				*val = std::stof(&raw[start], &start);
				return pos + start;
			}
			return 0;
		}

		static int ParseInt(std::string& raw, int pos, uint32_t* val)
		{
			size_t start = raw.find('(', pos) + 1;
			if (start >= 0)
			{
				*val = std::stoi(&raw[start], &start);
				return pos + start;
			}
			return 0;
		}
		
		static int ParseTotalConsumption1(Measurement& dsmr, std::string& raw, int pos)
		{
			return ParseFloat(raw, pos, &dsmr.TotalConsumption1);
		}

		static int ParseTotalConsumption2(Measurement& dsmr, std::string& raw, int pos)
		{
			return ParseFloat(raw, pos, &dsmr.TotalConsumption2);
		}

		static int ParseTotalProduction1(Measurement& dsmr, std::string& raw, int pos)
		{
			return ParseFloat(raw, pos, &dsmr.TotalProduction1);
		}

		static int ParseTotalProduction2(Measurement& dsmr, std::string& raw, int pos)
		{
			return ParseFloat(raw, pos, &dsmr.TotalProduction2);
		}

		static int ParseActualConsumption(Measurement& dsmr, std::string& raw, int pos)
		{
			return ParseFloat(raw, pos, &dsmr.ActualConsumption);
		}
		
		static int ParseActualProduction(Measurement& dsmr, std::string& raw, int pos)
		{
			return ParseFloat(raw, pos, &dsmr.ActualProduction);
		}
		
		static int ParseActualTarrif(Measurement& dsmr, std::string& raw, int pos)
		{
			return ParseInt(raw, pos, &dsmr.ActualTarrif);
		}

		static int ParseTotalGasConsumption(Measurement& dsmr, std::string& raw, int pos)
		{
			size_t start = raw.find('\n', pos) + 1;
			if (start >= 0)
			{
				return ParseFloat(raw, start, &dsmr.TotalGasConsumption);
			}
			return 0;
		}

		
		
		struct ValueParser
		{
			std::string key;
			int(*Parse)(Measurement& dsmr, std::string& raw, int start);
		};

		const std::vector<ValueParser> valueParsers {
			{"1-0:1.8.1", ParseTotalConsumption1},
			{"1-0:1.8.2", ParseTotalConsumption2},
			{"1-0:2.8.1", ParseTotalProduction1},
			{"1-0:2.8.1", ParseTotalProduction2},
			{"1-0:1.7.0", ParseActualConsumption},
			{"1-0:2.7.0", ParseActualProduction},
			{"0-1:24.3.0", ParseTotalGasConsumption},
			{"0-0:96.14.0", ParseActualTarrif}
			
		};


		void Parse(std::string& raw, Measurement& dsmr)
		{
			int ptr = 0;
			int len = raw.size();
			int pcnt = valueParsers.size();

			while (ptr < len && ptr >= 0)
			{
				bool parsed = false;
				for (int i = 0; i < pcnt; i++)
				{
					if (raw.rfind(valueParsers[i].key, ptr) == ptr) {
						int consumed = valueParsers[i].Parse(dsmr, raw, ptr);
						if (consumed > 0)
						{
							ptr = consumed + 1;
							parsed = true;
						}
					}
				}
				if (!parsed)
				{
					ptr = raw.find('\n', ptr);
					if (ptr >= 0)
						ptr++;
				}
			}
		}
	};
}