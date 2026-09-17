import os
from pptx import Presentation
from pptx.util import Inches, Pt
from pptx.dml.color import RGBColor

def create_deck():
    prs = Presentation()
    
    # Design colors
    NAVY = RGBColor(0, 122, 204)
    CHARCOAL = RGBColor(50, 50, 50)
    
    slides_data = [
        {
            "title": "Real-Time Biomechanical Tracking Framework",
            "subtitle": "Integrating Movella DOT Sensors with OpenSim SDK on Windows 11\nPresenter: Srushti",
            "bullets": []
        },
        {
            "title": "The Engineering Challenge",
            "subtitle": "",
            "bullets": [
                "Objective: Stream dual-arm data for joint angle estimation.",
                "The Roadblock: Custom Python scripts returned 0 Hz streams.",
                "OS Restrictions: Windows 11 aggressively filtered third-party BLE packets.",
                "Handshake Security: Newer Movella firmware ignores unauthenticated BLE requests."
            ]
        },
        {
            "title": "Root Cause Analysis & Pivots",
            "subtitle": "",
            "bullets": [
                "Mac vs Windows: Medium blog patterns failed on Windows architecture.",
                "Firmware v2 Shifts: Custom UUID streams required explicit descriptor triggers.",
                "The Solution: Transitioned from raw Bleak code to official SDK bindings.",
                "Environment Alignment: Resolved Python version mismatches via custom wheel packaging."
            ]
        },
        {
            "title": "System Integration Architecture",
            "subtitle": "",
            "bullets": [
                "IDE Workspace: Consolidated editing and execution within VS Code.",
                "Thread Safety: Implemented asyncio Queue arrays for background streams.",
                "C++ Backend: Loaded native Movella binary hooks via xdpchandler.py.",
                "Hardware Isolation: Filtered out ambient environmental sensors using address whitelists."
            ]
        },
        {
            "title": "Bridging to OpenSim SDK",
            "subtitle": "",
            "bullets": [
                "GUI Limitations: Bypassed frozen OpenSim desktop user interface configurations.",
                "C++ Binary Paths: Resolved simbody DLL lookup failures via runtime injection.",
                "Graphics Workaround: Disabled OpenGL visualization to prevent driver crashes.",
                "Biomechanical Mapping: Synchronized tracking streams to the native r_elbow_flex coordinate."
            ]
        },
        {
            "title": "Final Milestones & Success",
            "subtitle": "",
            "bullets": [
                "Antenna Stability: Sustained flawless multi-sensor connection streams.",
                "Live IK Calculation: Extracted real-time joint degree modifications at 60 Hz.",
                "Clean Fallbacks: Implemented thread-safe hardware socket closure.",
                "Next Steps: Exporting real-time kinematic arrays to motion trajectory logs."
            ]
        }
    ]

    for idx, data in enumerate(slides_data):
        if idx == 0:
            slide = prs.slides.add_slide(prs.slide_layouts[0])
            title = slide.shapes.title
            subtitle = slide.placeholders[1]
            title.text = data["title"]
            subtitle.text = data["subtitle"]
            title.text_frame.paragraphs[0].font.color.rgb = NAVY
        else:
            slide = prs.slides.add_slide(prs.slide_layouts[1])
            title = slide.shapes.title
            title.text = data["title"]
            title.text_frame.paragraphs[0].font.color.rgb = NAVY
            
            tf = slide.placeholders[1].text_frame
            tf.text = data["bullets"][0]
            for bullet in data["bullets"][1:]:
                p = tf.add_paragraph()
                p.text = bullet
                p.level = 0
                p.font.color.rgb = CHARCOAL

    output_path = "Movella_OpenSim_Integration.pptx"
    prs.save(output_path)
    print(f"[✔] SUCCESS: PowerPoint presentation generated -> '{output_path}'")

if __name__ == "__main__":
    create_deck()
