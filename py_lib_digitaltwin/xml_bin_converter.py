import xml.etree.ElementTree as ET

class XMLToBinaryConverter:
    @staticmethod
    def convert(xml_string):
        """
        Converts the given XML string to its corresponding binary representation.
        
        Args:
            xml_string (str): The XML string to convert.
        
        Returns:
            str: The extracted binary string.
        """
        try:
            root = ET.fromstring(xml_string)
            teds_code = root.find(".//TEDS-Code")
            if teds_code is not None:
                return teds_code.text.strip()
            
            security_data_block = root.find(".//SecurityTEDSDataBlock")
            if security_data_block is not None:
                # Construct binary string for SecurityTEDS
                level = security_data_block.find("Level").text
                num_of_standards = int(security_data_block.find("NumOfStandards").text)
                binary_data = f"6D400020040F000080A0000000000000 {level}{num_of_standards:02X}"
                for standard in security_data_block:
                    if standard.tag.startswith("SecurityStdName"):
                        std_name = standard.text
                        std_version_tag = f"SecurityStdVersion{standard.tag[-1]}"
                        std_version = security_data_block.find(std_version_tag).text
                        binary_data += f"{int(std_name):02X}{int(std_version):02X}"
                return binary_data
            
            raise ValueError("No valid binary data found in the XML.")
        except ET.ParseError as e:
            raise ValueError(f"Invalid XML format: {e}")

# Example usage
if __name__ == "__main__":
    TEMPTEDS = """<?xml version="1.0" encoding="UTF-8"?> <TEDS ID="[v03]"> <BasicTEDS> <Manufacturer>Dewesoft</Manufacturer> <Model>1</Model> <VersionLetter>A</VersionLetter> <VersionNumber>1</VersionNumber> <SerialNumber>1</SerialNumber> </BasicTEDS> <TEDS-Code length="40">6A4000200401000098D0421F00000000 00000012040000120400001204000000 C34800000000E000</TEDS-Code> <Info-Section EditorVersion="Dewesoft TedsEditor V2.2.12"> <InfoLine16>2023/05/22 0:25:06: Templates cleared by User.</InfoLine16> <InfoLine17>2023/05/22 0:25:28: Base template [#38] Thermistor created.</InfoLine17> </Info-Section> <DEBUG>TEMP</DEBUG> </TEDS>"""
    HUMIDTEDS = """<?xml version="1.0" encoding="UTF-8"?> <TEDS ID="[v03]"> <BasicTEDS> <Manufacturer>UnknownID(109)</Manufacturer> <Model>1</Model> <VersionLetter>A</VersionLetter> <VersionNumber>1</VersionNumber> <SerialNumber>2</SerialNumber> </BasicTEDS> <TEDS-Code length="31">6D4000200402000080A0000000000000 C8420000000000869100000000C001</TEDS-Code> <Info-Section EditorVersion="Dewesoft TedsEditor V2.2.12"> </Info-Section> <DEBUG>HUMID</DEBUG> </TEDS>"""
    SERVOTEDS = """<?xml version="1.0" encoding="UTF-8"?> <TEDS ID="[v03]"> <BasicTEDS> <Manufacturer>UnknownID(109)</Manufacturer> <Model>1</Model> <VersionLetter>A</VersionLetter> <VersionNumber>1</VersionNumber> <SerialNumber>3</SerialNumber> </BasicTEDS> <TEDS-Code length="69">6D4000200403000080A0000000000000 C8420000000000869100000000806AC0 82757A6CE560CDE450FA6C4D8B52EBB3 F9A43EA5050C00000280000034430300 00FE041C00</TEDS-Code> <Info-Section EditorVersion="Dewesoft TedsEditor V2.2.12"> <InfoLine16>2023/05/22 0:25:06: Templates cleared by User.</InfoLine16> <InfoLine17>2023/05/22 0:25:28: Base template [#38] Thermistor created.</InfoLine17> <InfoLine18>2023/05/22 0:25:38: TEDS write ChNr.1 started...</InfoLine18> <InfoLine19>2023/05/22 0:25:42: TEDS write to file [IEEE1451_4_106_1_A_1_1.tedsxml] done.</InfoLine19> <InfoLine22>2023/05/22 0:28:21: Templates cleared by User.</InfoLine22> </Info-Section> <DEBUG>SERVO</DEBUG> </TEDS>"""
    SECURITYTEDS = """<?xml version="1.0" encoding="UTF-8"?> <TEDS><SecurityTEDSDataBlock><Level>E</Level> <NumOfStandards>3</NumOfStandards> <SecurityStdName1>10</SecurityStdName1><SecurityStdVersion1>4</SecurityStdVersion1> <SecurityStdName2>12</SecurityStdName2><SecurityStdVersion2>1</SecurityStdVersion2> <SecurityStdName3>128</SecurityStdName3> <SecurityStdVersion3>1</SecurityStdVersion3> </SecurityTEDSDataBlock></TEDS>"""

    print(XMLToBinaryConverter.convert(TEMPTEDS))  # TEMPBINTEDS
    print(XMLToBinaryConverter.convert(HUMIDTEDS))  # HUMIDBINTEDS
    print(XMLToBinaryConverter.convert(SERVOTEDS))  # SERVOBINTEDS
    print(XMLToBinaryConverter.convert(SECURITYTEDS))  # SECURITYBINTEDS
