---
id: 1
title: docusaurus-ui-update
stage: spec
date_iso: 2025-12-25
surface: agent
model: Claude Sonnet 4.5
feature: docusaurus-ui-update
branch: 1-docusaurus-ui-update
user: saad
command: /sp.specify
labels: ["ui", "docusaurus", "specification"]
links:
  spec: "specs/1-docusaurus-ui-update/spec.md"
  ticket: null
  adr: null
  pr: null
files_yaml: |
  - specs/1-docusaurus-ui-update/spec.md
  - specs/1-docusaurus-ui-update/checklists/requirements.md
tests_yaml: |
  - spec creation
  - validation checklist
prompt_text: "# 📘 Physical AI Robotics Book – Professional UI Update Specification

## 🎯 Objective
Upgrade the UI of the existing Docusaurus-based book to a clean, modern, and professional layout suitable for a technical education book.

The content structure already exists. This task focuses **only on UI/UX improvements**, navigation, layout, and styling — not content rewriting.

---

## 🧭 Layout Structure

### 1️⃣ Top Navigation (Tabs-Based)
Create **4 main tabs** in the top navigation bar:

- Module 1
- Module 2
- Module 3
- Module 4

---

### 2️⃣ Main / Home Tab (Default View)
The **main tab** should behave like the current book view:

- All modules (1–4) visible together
- Same order as existing book
- Scroll-based reading experience
- No content removal

This acts as the **full book view**

---

### 3️⃣ Module Tabs Behavior
When clicking on a module tab:

- Only the selected module opens
- Other modules are hidden
- Sidebar updates to show **only that module's chapters**
- Breadcrumb navigation enabled:
  - Home → Module X → Chapter

---

## 🎨 UI Design & Styling

### Color Palette (Professional & Minimal)
- Primary Color: `#0F172A` (Dark Navy / Slate)
- Secondary Color: `#2563EB` (Muted Blue)
- Accent Color: `#38BDF8` (Soft Sky Blue)
- Background:
  - Light mode: `#F8FAFC`
  - Dark mode: `#020617`
- Text:
  - Headings: `#020617`
  - Body: `#334155`

---

### Typography
- Headings: **Inter / Poppins / System Sans**
- Body Text: **Inter / Roboto**
- Code Blocks:
  - Monospace font
  - Soft background
  - Rounded corners

---

## 📚 Sidebar Improvements
- Collapsible sections
- Active chapter highlighted
- Smooth expand/collapse animation
- Module-wise filtering when a module tab is selected

---

## ✨ UI Enhancements
- Soft shadows on cards
- Rounded corners (8–12px)
- Smooth hover effects
- Clear visual hierarchy
- Improved spacing and readability
- Responsive layout (Desktop + Tablet + Mobile)

---

## 🦶 Footer (Strict Requirement)
Footer must contain **ONLY** this text:"
response_text: "Created specification for Docusaurus UI update for Physical AI Robotics Book. Created branch 1-docusaurus-ui-update with spec file at specs/1-docusaurus-ui-update/spec.md and validation checklist at specs/1-docusaurus-ui-update/checklists/requirements.md. The spec includes user scenarios, functional requirements, and success criteria for implementing tab-based navigation, professional UI styling, and improved sidebar functionality."
outcome: "Specification created successfully with all required sections. Quality checklist completed with all items validated."
evaluation: "The specification meets all requirements and is ready for the planning phase (/sp.plan)."
---

# 📘 Physical AI Robotics Book – Professional UI Update Specification

## 🎯 Objective
Upgrade the UI of the existing Docusaurus-based book to a clean, modern, and professional layout suitable for a technical education book.

The content structure already exists. This task focuses **only on UI/UX improvements**, navigation, layout, and styling — not content rewriting.

---

## 🧭 Layout Structure

### 1️⃣ Top Navigation (Tabs-Based)
Create **4 main tabs** in the top navigation bar:

- Module 1
- Module 2
- Module 3
- Module 4

---

### 2️⃣ Main / Home Tab (Default View)
The **main tab** should behave like the current book view:

- All modules (1–4) visible together
- Same order as existing book
- Scroll-based reading experience
- No content removal

This acts as the **full book view**

---

### 3️⃣ Module Tabs Behavior
When clicking on a module tab:

- Only the selected module opens
- Other modules are hidden
- Sidebar updates to show **only that module's chapters**
- Breadcrumb navigation enabled:
  - Home → Module X → Chapter

---

## 🎨 UI Design & Styling

### Color Palette (Professional & Minimal)
- Primary Color: `#0F172A` (Dark Navy / Slate)
- Secondary Color: `#2563EB` (Muted Blue)
- Accent Color: `#38BDF8` (Soft Sky Blue)
- Background:
  - Light mode: `#F8FAFC`
  - Dark mode: `#020617`
- Text:
  - Headings: `#020617`
  - Body: `#334155`

---

### Typography
- Headings: **Inter / Poppins / System Sans**
- Body Text: **Inter / Roboto**
- Code Blocks:
  - Monospace font
  - Soft background
  - Rounded corners

---

## 📚 Sidebar Improvements
- Collapsible sections
- Active chapter highlighted
- Smooth expand/collapse animation
- Module-wise filtering when a module tab is selected

---

## ✨ UI Enhancements
- Soft shadows on cards
- Rounded corners (8–12px)
- Smooth hover effects
- Clear visual hierarchy
- Improved spacing and readability
- Responsive layout (Desktop + Tablet + Mobile)

---

## 🦶 Footer (Strict Requirement)
Footer must contain **ONLY** this text: