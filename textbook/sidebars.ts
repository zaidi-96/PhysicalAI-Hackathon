import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  // YEH APNA MAIN SIDEBAR HAI
  tutorialSidebar: [
    // HOME
    {
      type: 'doc',
      id: 'intro',
      label: '🏠 Home',
    },
    
    // MODULE 1
    {
      type: 'doc',
      id: 'module-1-ros2',
      label: '🤖 Module 1: ROS 2 Basics',
    },
    
    // MODULE 2  
    {
      type: 'doc',
      id: 'module-2-gazebo',
      label: '🖥️ Module 2: Gazebo Simulation',
    },
    
    // MODULE 3 - NAME CHANGE KARO
    {
      type: 'doc',
      id: 'module-3-nvidia-isaac',  // ✅ YEH SAHI NAME HAI
      label: '⚡ Module 3: NVIDIA Isaac',
    },
    
    // MODULE 4 - NAME CHANGE KARO  
    {
      type: 'doc',
      id: 'module-4-vlm',  // ✅ YEH SAHI NAME HAI
      label: '🧠 Module 4: VLM Models',
    },
    
    // MODULE 5 - NAME CHANGE KARO
    {
      type: 'doc',
      id: 'module-5-capstone',  // ✅ YEH SAHI NAME HAI
      label: '🏆 Module 5: Capstone Project',
    },
    
    // TUTORIAL FILES HATA DO (Optional)
    /*
    {
      type: 'doc',
      id: 'tutorial-basics/create-a-document',
      label: 'Tutorial: Create Document',
    },
    */
  ],
};

export default sidebars;