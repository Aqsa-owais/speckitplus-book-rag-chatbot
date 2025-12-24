# Data Model: Docusaurus UI Update for Physical AI Robotics Book

**Feature**: Docusaurus UI Update for Physical AI Robotics Book
**Date**: 2025-12-25
**Branch**: 1-docusaurus-ui-update

## Entities

### Module
- **Description**: Represents a major section of the book containing multiple chapters
- **Properties**:
  - id: string (unique identifier for the module)
  - title: string (display name for the module, e.g. "Module 1")
  - chapters: Chapter[] (list of chapters contained in the module)
  - order: number (position in the sequence of modules)

### Chapter
- **Description**: Represents a subsection within a module
- **Properties**:
  - id: string (unique identifier for the chapter)
  - title: string (display name for the chapter)
  - path: string (URL path to the chapter)
  - module: Module (reference to the parent module)
  - order: number (position in the sequence within the module)

### TabState
- **Description**: Represents the current state of the tab navigation system
- **Properties**:
  - activeTab: string ("home" or module ID)
  - visibleModules: Module[] (modules currently displayed based on tab selection)
  - sidebarContent: object (content to display in sidebar based on active tab)

### NavigationItem
- **Description**: Represents an item in the navigation system
- **Properties**:
  - id: string (unique identifier)
  - title: string (display text)
  - path: string (URL path)
  - type: string ("module", "chapter", "link")
  - children: NavigationItem[] (sub-items if applicable)

### ThemeConfig
- **Description**: Represents the styling configuration for the UI
- **Properties**:
  - colors: object (color palette with primary, secondary, accent, etc.)
  - typography: object (font families for headings and body)
  - spacing: object (spacing rules)
  - borderRadius: number (value in pixels for rounded corners)
  - shadows: object (shadow definitions)

## Relationships

- Module contains multiple Chapters
- TabState references one or more Modules
- NavigationItem can contain other NavigationItems (hierarchical)
- ThemeConfig applies to all UI components

## State Transitions

### Tab Selection
- **Initial State**: Home tab active, all modules visible
- **Action**: User clicks on a module tab
- **Transition**: Active tab changes to selected module, only that module's content is shown
- **Result**: Sidebar updates to show only chapters from selected module

### Module Navigation
- **Initial State**: Specific module tab active
- **Action**: User clicks on home tab
- **Transition**: Active tab changes to home, all modules become visible
- **Result**: Sidebar updates to show all modules and chapters