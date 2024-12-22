import xml.etree.ElementTree as ET
from xml.dom import minidom

def coords_to_gpx_route(coords, filename=None):
    """
    Converts a list of (latitude, longitude) tuples into a GPX route.

    Parameters:
    - coords: List of tuples, where each tuple contains (latitude, longitude).
    - filename: Optional; if provided, the GPX data will be written to this file.

    Returns:
    - A string containing the GPX data if filename is not provided.
    """
    # Define the root element with necessary attributes
    gpx_attrs = {
        'version': '1.1',
        'creator': 'coords_to_gpx_route_function',
        'xmlns': 'http://www.topografix.com/GPX/1/1',
        'xmlns:xsi': 'http://www.w3.org/2001/XMLSchema-instance',
        'xsi:schemaLocation': ('http://www.topografix.com/GPX/1/1 '
                               'http://www.topografix.com/GPX/1/1/gpx.xsd')
    }
    gpx = ET.Element('gpx', gpx_attrs)
    
    # Create the route element
    rte = ET.SubElement(gpx, 'rte')
    
    # Iterate over the coordinates and create route points
    for lat, lon in coords:
        rtept_attrs = {'lat': f'{lat}', 'lon': f'{lon}'}
        ET.SubElement(rte, 'rtept', rtept_attrs)
    
    # Generate the XML string
    rough_string = ET.tostring(gpx, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    gpx_string = reparsed.toprettyxml(indent="  ")
    
    # Write to file if filename is provided
    if filename:
        with open(filename, 'w', encoding='utf-8') as f:
            f.write(gpx_string)
    else:
        return gpx_string
        