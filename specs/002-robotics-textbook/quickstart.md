# Quickstart Guide: "Physical AI & Humanoid Robotics" Textbook

This guide provides instructions on how to set up, build, and view the "Physical AI & Humanoid Robotics" textbook locally using Docusaurus.

## Prerequisites

Before you begin, ensure you have the following installed:

*   **Node.js**: Version 18 or higher. You can download it from [nodejs.org](https://nodejs.org/).
*   **npm** (Node Package Manager): Usually comes bundled with Node.js.
*   **Git**: For cloning the repository.

## 1. Clone the Repository

First, clone the project repository to your local machine:

```bash
git clone <repository_url_here>
cd Docusaurus-frontend
```

**Note**: Replace `<repository_url_here>` with the actual URL of your project's GitHub repository.

## 2. Install Dependencies

Navigate into the `Docusaurus-frontend` directory and install the necessary Node.js dependencies:

```bash
cd Docusaurus-frontend
npm install
```

## 3. Start the Development Server

Once the dependencies are installed, you can start the Docusaurus development server. This will build the site and serve it locally, automatically reloading when you make changes.

```bash
npm run dev
```

This command typically opens your browser to `http://localhost:3000`. If it doesn't, manually navigate to that URL in your web browser.

## 4. Build for Production

To create a production-ready static build of your Docusaurus site, use the following command:

```bash
npm run build
```

This will generate static HTML, CSS, and JavaScript files in the `build` directory within your `Docusaurus-frontend` folder. These files can then be deployed to any static hosting service, such as GitHub Pages.

## 5. View Production Build Locally (Optional)

To serve your production build locally and ensure everything looks correct before deployment, you can use a static server:

```bash
npm install -g serve
serve -s build
```

Then, open your browser to `http://localhost:3000` (or the port specified by the `serve` command).

---

**Next Steps**: Once the textbook content is generated (Phase 1 completion), it will populate the `docs` directory within `Docusaurus-frontend/`. You can then follow these steps to preview the full textbook.